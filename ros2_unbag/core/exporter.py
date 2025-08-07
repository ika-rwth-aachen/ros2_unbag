# MIT License

# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from collections import defaultdict, deque
from datetime import datetime
import logging
import multiprocessing as mp
from pathlib import Path
import threading

from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.file_utils import get_time_from_msg


class Exporter:
    # Handles parallel export of messages from a ROS2 bag

    def __init__(self, bag_reader, export_config, global_config, progress_callback=None):
        """
        Initialize Exporter with bag reader, export and global configs.
        Set up topic indexing, worker count, queue size, and optional progress callback.

        Args:
            bag_reader: BagReader instance for the ROS2 bag.
            export_config: Dict of per-topic export configuration.
            global_config: Dict of global settings.
            progress_callback: Optional function for reporting progress.

        Returns:
            None
        """
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

        self.bag_reader = bag_reader
        self.config = export_config
        self.global_config = global_config
        self.topic_types = self.bag_reader.topic_types
        self.progress_callback = progress_callback

        # check if topic exists in the bag, if not, raise an error and list all available topics
        for topic in self.config:
            if topic not in self.topic_types:
                raise ValueError(f"Topic '{topic}' not found in bag. Available topics: {list(self.topic_types.keys())}")

        self.index_map = {t: 0 for t in self.config}
        self.sequential_topics = [t for t, c in self.config.items()
            if ExportRoutine.get_mode(self.topic_types[t], c['format']) == ExportMode.SINGLE_FILE
        ]

        # one queue for parallel topics
        self.parallel_q = mp.Queue()
        # one queue per sequential topic
        self.seq_queues = {t: mp.Queue() for t in self.sequential_topics}

        self.num_workers = max(1, int(mp.cpu_count() * self.global_config["cpu_percentage"] * 0.01))
        self.num_parallel_workers = max(1, self.num_workers - len(self.sequential_topics))

        self.logger.info(f"Using {self.num_workers} workers for export, "
              f"{self.num_parallel_workers} for parallel topics, "
              f"{len(self.sequential_topics)} for sequential topics.")
        self.queue_maxsize = self.num_workers * 2
        self._enqueued_files = set()

        # Pre-fetch export handlers and processors
        self.topic_handlers = {}
        self.topic_processors = {}

        for topic, cfg in self.config.items():

            fmt = cfg['format']
            topic_type = self.topic_types[topic]

            # Export handler
            self.topic_handlers[topic] = ExportRoutine.get_handler(topic_type, fmt)

            # Optional processor
            if 'processor' in cfg:
                proc_name = cfg['processor']
                proc_args = cfg.get('processor_args', {})
                required_args = Processor.get_required_args(topic_type, proc_name)
                missing_args = [arg for arg in required_args if arg not in proc_args]
                if missing_args:
                    raise ValueError(
                        f"Missing required arguments for processor '{proc_name}': {', '.join(missing_args)}"
                    )
                proc_handler = Processor.get_handler(topic_type, proc_name)
                self.topic_processors[topic] = (proc_handler, proc_args)
            else:
                self.topic_processors[topic] = None


    def run(self):
        """
        Orchestrate parallel export: configure reader, start producer, workers, and monitor.
        Handle exceptions, clean shutdown, and report progress via callback.

        Args:
            None

        Returns:
            None

        Raises:
            RuntimeError: If an exception occurs in a worker or producer.
            KeyboardInterrupt: If interrupted by user.
        """
        # Start export process using multiprocessing
        self.message_count = self.bag_reader.get_message_count()
        self.max_progress_count = sum(
            self.message_count.get(key, 0) for key in self.config)
        self.bag_reader.set_filter(self.config.keys())

        # Queues for exceptions and progress
        self.exception_queue = mp.Queue()
        progress_queue = mp.Queue()

        # Start producer process to generate tasks
        producer = mp.Process(target=self._producer, name="Producer", daemon=True)
        producer.start()

        # Start worker processes
        workers = []

        # Start workers for parallel topics
        for i in range(self.num_parallel_workers):
            process = mp.Process(target=self._worker, args=(self.parallel_q, progress_queue), name=f"Par-{i}", daemon=True)
            process.start(); workers.append(process)

        # Start one worker per sequential topic
        for topic, q in self.seq_queues.items():
            process = mp.Process(target=self._worker, args=(q, progress_queue, self.parallel_q), name=f"Seq-{topic}", daemon=True)
            process.start(); workers.append(process)

        # Start monitor thread to update progress
        monitor = threading.Thread(target=self._monitor,
                                   args=(progress_queue,),
                                   name="Monitor",
                                   daemon=True)
        monitor.start()

        # Monitor the queues and handle exceptions
        try:
            while True:
                if not self.exception_queue.empty():
                    # If an exception occurred, retrieve it and terminate all processes
                    exc_type, exc_msg = self.exception_queue.get()

                    producer.terminate()                    
                    for w in workers:
                        w.terminate()

                    producer.join()
                    for w in workers:
                        w.join()

                    raise RuntimeError(f"[{exc_type}] {exc_msg}")

                if not producer.is_alive() and all(not w.is_alive() for w in workers):
                    break

        except KeyboardInterrupt:
            self.logger.warning("Keyboard interrupt detected. Cleaning up...")
            producer.terminate()
            for w in workers:
                w.terminate()
            producer.join()
            for w in workers:
                w.join()
            raise

        progress_queue.put(None)
        monitor.join()


    def abort_export(self):
        """
        Abort export by throwing a user abort exception.

        Args:
            None

        Returns:
            None
        """
        error = RuntimeError(f"Export aborted by user")
        self.exception_queue.put((type(error).__name__, str(error)))


    def _producer(self):
        """
        Read messages, apply optional resampling strategy, enqueue export tasks, track dropped frames, and signal workers.

        Args:
            None

        Returns:
            None
        """
        try:
            dropped_frames = defaultdict(int)  # topic -> count

            # Get resampling config: master topic, association strategy, and discard threshold
            master_topic, assoc_strategy, discard_eps = self._get_resampling_config()

            if master_topic is None:
                # No resampling configured – export all messages individually
                self._export_all_messages()
                return

            # Dispatch to the appropriate resampling strategy
            if assoc_strategy == 'last':
                self._process_last_association(master_topic, discard_eps, dropped_frames)
            elif assoc_strategy == 'nearest':
                self._process_nearest_association(master_topic, discard_eps, dropped_frames)

            # Output summary and clean exit
            self._print_drop_summary(dropped_frames)
            self._signal_worker_termination()

        except Exception as e:
            self.exception_queue.put((type(e).__name__, str(e)))
            self._signal_worker_termination()
            

    def _get_resampling_config(self):
        """
        Scan config and extract master topic and resampling strategy.
        Only one master topic is allowed.

        Args:
            None

        Returns:
            tuple: (master_topic: str or None, assoc_strategy: str or None, discard_eps: float or None)
        """
        global_rcfg = self.global_config.get("resample_config")
        if global_rcfg:
            master = global_rcfg.get("master_topic")
            if master and master not in self.config:
                raise ValueError(f"Master topic '{master}' not found in export config.")
            if not master:
                raise ValueError("Resample_config must define a 'master_topic'")
            assoc = global_rcfg.get("association", "last")
            discard_eps = global_rcfg.get("discard_eps")
            if assoc == "nearest" and discard_eps is None:
                raise ValueError("'nearest' association requires 'discard_eps' in global config.")
            self.logger.info(f"Resampling with strategy '{assoc}' to master topic '{master}'")
            return master, assoc, discard_eps
        return None, None, None


    def _export_all_messages(self):
        """
        Read and enqueue every message from configured topics without resampling, then signal workers to terminate.

        Args:
            None

        Returns:
            None
        """
        while True:
            res = self.bag_reader.read_next_message()
            if res is None:
                break
            topic, msg, _ = res
            if topic in self.config:
                self._enqueue_export_task(topic, msg)
        self._signal_worker_termination()


    def _process_last_association(self, master_topic, discard_eps,
                                  dropped_frames):
        """
        Resampling strategy: 'last'.
        Collect the latest message from each topic and align frames based on latest state when master message arrives.

        Args:
            master_topic: Topic name to use as master (str).
            discard_eps: Optional float threshold for discarding frames.
            dropped_frames: Dict for tracking dropped frames per topic.

        Returns:
            None
        """
        latest_messages = {}
        latest_ts_seen = 0.0

        while True:
            res = self.bag_reader.read_next_message()
            if res is None:
                break

            topic, msg, _ = res
            cfg = self.config.get(topic)
            if not cfg:
                continue

            ts = get_time_from_msg(msg, return_datetime=False)

            latest_ts_seen = max(latest_ts_seen, ts)
            latest_messages[topic] = (ts, msg)

            if topic != master_topic:
                continue  # Wait for master message

            master_ts = ts
            frame = {}

            # Attempt to assemble a complete frame
            for t in self.config:
                if t == master_topic:
                    frame[t] = msg
                    continue
                if t not in latest_messages:
                    frame = None
                    break
                sel_ts, sel_msg = latest_messages[t]
                if discard_eps is not None and abs(master_ts -
                                                   sel_ts) > discard_eps:
                    frame = None
                    break
                frame[t] = sel_msg

            if frame:
                for t, m in frame.items():
                    self._enqueue_export_task(t, m)
            else:
                for t in self.config:
                    if t == master_topic:
                        continue
                    if t not in latest_messages or (discard_eps and abs(
                            master_ts - latest_messages[t][0]) > discard_eps):
                        dropped_frames[t] += 1


    def _process_nearest_association(self, master_topic,
                                     discard_eps, dropped_frames):
        """
        Resampling strategy: 'nearest'.
        Buffer all messages and, when a master message arrives, find the closest message from each other topic.

        Args:
            master_topic: Topic name to use as master (str).
            discard_eps: Float threshold for discarding frames.
            dropped_frames: Dict for tracking dropped frames per topic.

        Returns:
            None
        """
        buffers = defaultdict(deque)
        latest_ts_seen = 0.0

        while True:
            res = self.bag_reader.read_next_message()
            if res is None:
                break

            topic, msg, _ = res
            cfg = self.config.get(topic)
            if not cfg:
                continue

            ts = get_time_from_msg(msg, return_datetime=False)
            
            latest_ts_seen = max(latest_ts_seen, ts)
            buffers[topic].append((ts, msg))

            if topic != master_topic:
                continue

            # Attempt to process all buffered master messages up to current threshold
            while buffers[master_topic]:
                candidate_ts, candidate_msg = buffers[master_topic][0]
                if candidate_ts + discard_eps > latest_ts_seen:
                    break  # Delay until more data arrives

                master_ts = candidate_ts
                frame = {master_topic: candidate_msg}
                valid = True

                # Find best match from each topic
                for t in self.config:
                    if t == master_topic:
                        continue
                    candidates = [(ts_, msg_)
                                  for ts_, msg_ in buffers[t]
                                  if abs(ts_ - master_ts) <= discard_eps]
                    if not candidates:
                        valid = False
                        break
                    selected_ts, selected_msg = min(
                        candidates, key=lambda x: abs(x[0] - master_ts))
                    frame[t] = selected_msg

                if valid:
                    for t, m in frame.items():
                        self._enqueue_export_task(t, m)
                else:
                    for t in self.config:
                        if t == master_topic:
                            continue
                        if not any(
                                abs(ts_ - master_ts) <= discard_eps
                                for ts_, _ in buffers[t]):
                            dropped_frames[t] += 1

                # Remove processed master message
                buffers[master_topic].popleft()

            # Remove stale messages from buffers
            expire_before = latest_ts_seen - discard_eps * 2
            for t in buffers:
                while buffers[t] and buffers[t][0][0] < expire_before:
                    buffers[t].popleft()

    def _signal_worker_termination(self):
        """
        Signal worker threads to terminate by pushing sentinel values.

        Args:
            None

        Returns:
            None
        """
        # for the parallel pool
        for _ in range(self.num_parallel_workers):
            self.parallel_q.put(None)
        # for each sequential-topic worker
        for topic in self.sequential_topics:
            self.parallel_q.put(None)
            self.seq_queues[topic].put(None)


    def _print_drop_summary(self, dropped_frames):
        """
        Print summary of how many frames were dropped per topic.

        Args:
            dropped_frames: Dict mapping topic names to dropped frame counts.

        Returns:
            None
        """
        if not dropped_frames:
            return
        self.logger.info("The syncronization process dropped frames caused by the discard eps.\n"
        "The following topics were not available at frame generation time:")
        for topic, count in dropped_frames.items():
            self.logger.info(f"  {topic}: {count} times")


    def _enqueue_export_task(self, topic, msg):
        """
        Build filename and directory for a topic message, create path, and enqueue the export task with format.

        Args:
            topic: Topic name (str).
            msg: ROS2 message instance.

        Returns:
            None
        """
        cfg = self.config.get(topic)
        if not cfg:
            return

        fmt = cfg['format']
        path = cfg['path']
        subfolder = cfg.get('subfolder', "").strip("/")
        naming = cfg['naming']
        index = self.index_map[topic]
        self.index_map[topic] += 1

        topic_base = topic.strip("/").replace("/", "_")
        
        # Apply naming pattern
        replacements = {
            "%name": topic_base,
            "%index": str(index)
        }

        for key, value in replacements.items():
            naming = naming.replace(key, value)
            path = path.replace(key, value)
            subfolder = subfolder.replace(key, value)

        # Check if the naming still contains a placeholder
        if "%" in naming:
            # Build timestamp for filename
            timestamp = get_time_from_msg(msg, return_datetime=True)
            filename = timestamp.strftime(naming)
        else:
            filename = naming

        path = Path(path) / subfolder
        path.mkdir(parents=True, exist_ok=True)
        full_path = path / filename

        # Determine if this is the first time this file is being enqueued
        is_first = full_path not in self._enqueued_files
        self._enqueued_files.add(full_path)

        # Warn the user if the export routine is single-file and the file already exists
        topic_type = self.topic_types[topic]
        export_mode = ExportRoutine.get_mode(topic_type, fmt)
        sequential = topic in self.sequential_topics
        if export_mode == ExportMode.MULTI_FILE and not is_first:
            raise ValueError(f"Cannot use a non-changing file name for topic '{topic}' "
                             f"and format '{fmt}'. This will overwrite the previous file: {full_path}")
        
        # Create metadata for the export
        metadata = ExportMetadata(index=index, max_index=self.message_count[topic]-1)

        task = (topic, msg, full_path, fmt, metadata)
        if sequential:
            self.seq_queues[topic].put(task)
        else:
            self.parallel_q.put(task)


    def _worker(self, task_queue, progress_queue, fallback_queue=None):
        """
        Consume tasks, apply optional processor, invoke export routine, report progress, and forward exceptions.
        If a fallback queue is provided, the worker switches to it when the main queue is closed.

        Args:
            task_queue: Multiprocessing queue for export tasks.
            progress_queue: Multiprocessing queue for progress tokens.
            fallback_queue: Optional fallback queue for tasks if the main queue is closed.

        Returns:
            None
        """
        # Processes messages and performs export
        while True:
            task = task_queue.get()
            try:
                if task is None:
                    # if we have a fallback, switch to it; else, exit
                    if fallback_queue:
                        task_queue, fallback_queue = fallback_queue, None
                        continue
                    break
                topic, msg, full_path, fmt, metadata = task

                # Use pre-fetched processor if available
                processor = self.topic_processors.get(topic)
                if processor:
                    handler, args = processor
                    msg = handler(msg=msg, **args)

                # Use pre-fetched export handler
                export_handler = self.topic_handlers[topic]
                if export_handler:
                    export_handler(msg, full_path, fmt, metadata)
                    progress_queue.put(1)

            except Exception as e:
                # Handle exceptions during export
                self.exception_queue.put((type(e).__name__, str(e)))
                break


    def _monitor(self, progress_queue):
        """
        Count completed exports from progress tokens and invoke the progress callback until termination sentinel.

        Args:
            progress_queue: Multiprocessing queue for progress tokens.

        Returns:
            None
        """
        # Tracks and reports export progress
        done = 0
        while True:
            token = progress_queue.get()
            if token is None:
                break
            done += token
            if self.progress_callback:
                try:
                    self.progress_callback(done, self.max_progress_count)
                except Exception:
                    # Handle exceptions in progress callback
                    self.logger.error(f"Error in progress callback: {done}/{self.max_progress_count}")
                    pass
