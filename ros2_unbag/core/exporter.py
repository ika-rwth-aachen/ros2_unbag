from collections import defaultdict, deque
from datetime import datetime
import multiprocessing as mp
import os
import threading

from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.routines.base import ExportRoutine


class Exporter:
    # Handles parallel export of messages from a ROS2 bag

    def __init__(self, bag_reader, export_config, global_config, progress_callback=None):
        """
        Initialize Exporter with bag reader, export and global configs.
        Set up topic indexing, worker count, queue size, and optional progress callback.
        """
        self.bag_reader = bag_reader
        self.config = export_config
        self.index_map = {
            topic: 0 for topic in self.config
        }  # message index per topic
        self.topic_types = self.bag_reader.topic_types
        self.progress_callback = progress_callback
        self.num_workers = int(mp.cpu_count() * global_config["cpu_percentage"] * 0.01)
        if self.num_workers < 1:
            self.num_workers = 1
        self.queue_maxsize = self.num_workers * 2  # limit for task queue

    def run(self):
        """
        Orchestrate parallel export: configure reader, start producer, workers, and monitor.
        Handle exceptions, clean shutdown, and report progress via callback.
        """
        # Start export process using multiprocessing
        message_count = self.bag_reader.get_message_count()
        self.max_progress_count = sum(
            message_count.get(key, 0) for key in self.config)
        self.bag_reader.set_filter(self.config.keys())

        # Queues for tasks and progress tracking
        task_queue = mp.Queue(self.queue_maxsize)
        progress_queue = mp.Queue()
        self.exception_queue = mp.Queue()

        # Start producer process to generate tasks
        producer = mp.Process(target=self._producer,
                              args=(task_queue,),
                              name="Producer",
                              daemon=True)
        producer.start()

        # Start worker processes
        workers = []
        for wid in range(self.num_workers):
            worker = mp.Process(target=self._worker,
                                args=(task_queue, progress_queue),
                                name=f"Worker-{wid}",
                                daemon=True)
            worker.start()
            workers.append(worker)

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
            print("Keyboard interrupt detected. Cleaning up...")
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
        Abort export by throwing a user abort exception
        """
        error = RuntimeError(f"Export aborted by user")
        self.exception_queue.put((type(error).__name__, str(error)))

    def _producer(self, task_queue):
        """
        Read messages, apply optional resampling strategy, enqueue export tasks, track dropped frames, and signal workers.
        """
        try:
            dropped_frames = defaultdict(int)  # topic -> count

            # Get resampling config: master topic, association strategy, and discard threshold
            master_topic, assoc_strategy, discard_eps = self._get_resampling_config(
            )

            if master_topic is None:
                # No resampling configured – export all messages individually
                self._export_all_messages(task_queue)
                return

            # Dispatch to the appropriate resampling strategy
            if assoc_strategy == 'last':
                self._process_last_association(task_queue, master_topic,
                                            discard_eps, dropped_frames)
            elif assoc_strategy == 'nearest':
                self._process_nearest_association(task_queue, master_topic,
                                                discard_eps, dropped_frames)

            # Output summary and clean exit
            self._print_drop_summary(dropped_frames)
            self._signal_worker_termination(task_queue)

        except Exception as e:
            self.exception_queue.put((type(e).__name__, str(e)))
            self._signal_worker_termination(task_queue)
            

    def _get_resampling_config(self):
        """
        Scan config and extract master topic and resampling strategy.
        Only one master topic is allowed.
        """
        for topic, cfg in self.config.items():
            rcfg = cfg.get('resample_config')
            if rcfg and rcfg.get('is_master', False):
                print(
                    f"Warning: Topic '{topic}' is marked as master. Remember, that only one master topic is supported."
                )
                assoc_strategy = rcfg.get('association', 'last')
                discard_eps = rcfg.get('discard_eps')
                if assoc_strategy == 'nearest' and discard_eps is None:
                    raise ValueError(
                        f"'nearest' association requires 'discard_eps' for topic '{topic}'."
                    )
                return topic, assoc_strategy, discard_eps
        return None, None, None

    def _export_all_messages(self, task_queue):
        """
        Read and enqueue every message from configured topics without resampling, then signal workers to terminate.
        """
        while True:
            res = self.bag_reader.read_next_message()
            if res is None:
                break
            topic, msg, _ = res
            if topic in self.config:
                self._enqueue_export_task(topic, msg, task_queue)
        self._signal_worker_termination(task_queue)

    def _process_last_association(self, task_queue, master_topic, discard_eps,
                                  dropped_frames):
        """
        Resampling strategy: 'last'.
        Collect the latest message from each topic and align frames based on latest state when master message arrives.
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

            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
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
                    self._enqueue_export_task(t, m, task_queue)
            else:
                for t in self.config:
                    if t == master_topic:
                        continue
                    if t not in latest_messages or (discard_eps and abs(
                            master_ts - latest_messages[t][0]) > discard_eps):
                        dropped_frames[t] += 1

    def _process_nearest_association(self, task_queue, master_topic,
                                     discard_eps, dropped_frames):
        """
        Resampling strategy: 'nearest'.
        Buffer all messages and, when a master message arrives, find the closest message from each other topic.
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

            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
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
                        self._enqueue_export_task(t, m, task_queue)
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

    def _signal_worker_termination(self, task_queue):
        """
        Signal worker threads to terminate by pushing sentinel values.
        """
        for _ in range(self.num_workers):
            task_queue.put(None)

    def _print_drop_summary(self, dropped_frames):
        """
        Print summary of how many frames were dropped per topic.
        """
        if not dropped_frames:
            return
        print("\nDropped frames per topic:")
        for topic, count in dropped_frames.items():
            print(f"  {topic}: {count}")

    def _enqueue_export_task(self, topic, msg, task_queue):
        """
        Build filename and directory for a topic message, create path, and enqueue the export task with format.
        """
        cfg = self.config.get(topic)
        if not cfg:
            return

        fmt = cfg['format']
        path = cfg['path']
        naming = cfg['naming']
        index = self.index_map[topic]
        self.index_map[topic] += 1

        topic_base = topic.strip("/").replace("/", "_")

        # Build timestamp for filename
        try:
            timestamp = datetime.fromtimestamp(msg.header.stamp.sec +
                                               msg.header.stamp.nanosec * 1e-9)
        except AttributeError:
            # Fallback timestamp (receive time)
            timestamp = datetime.fromtimestamp(msg.stamp.sec +
                                               msg.stamp.nanosec * 1e-9)

        # Apply naming pattern
        replacements = {
            "%name": topic_base,
            "%index": str(index),
            "%ros_timestamp": self._format_ros_timestamp(msg.header) if hasattr(msg, "header") else ""
        }

        for key, value in replacements.items():
            naming = naming.replace(key, value)

        filename = timestamp.strftime(naming)

        os.makedirs(path, exist_ok=True)
        full_path = os.path.join(path, filename)
        task_queue.put((topic, msg, full_path, fmt))

    def _worker(self, task_queue, progress_queue):
        """
        Consume tasks, apply optional processor, invoke export routine, report progress, and forward exceptions.
        """
        # Processes messages and performs export
        while True:
            task = task_queue.get()
            try:
                if task is None:
                    break
                topic, msg, full_path, fmt = task

                # Check if the topic has a processor defined
                if 'processor' in self.config[topic]:
                    topic_type = self.topic_types[topic]
                    handler = Processor.get_handler(
                        topic_type, self.config[topic]['processor'])
                    if handler:
                        processor_args = self.config[topic].get(
                            'processor_args', {})
                        required_args = Processor.get_required_args(
                            topic_type, self.config[topic]['processor'])

                        # Check if all required arguments are provided
                        missing_args = [
                            arg for arg in required_args
                            if arg not in processor_args
                        ]
                        if missing_args:
                            raise ValueError(
                                f"Missing required arguments for processor '{self.config[topic]['processor']}': {', '.join(missing_args)}"
                            )

                        msg = handler(msg=msg, **processor_args)

                topic_type = self.topic_types[topic]
                handler = ExportRoutine.get_handler(topic_type, fmt)
                if handler:
                    handler(msg, full_path, fmt)
                    progress_queue.put(1)
            except Exception as e:
                # Handle exceptions during export
                self.exception_queue.put((type(e).__name__, str(e)))
                break

    def _monitor(self, progress_queue):
        """
        Count completed exports from progress tokens and invoke the progress callback until termination sentinel.
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
                    print(f"Error in progress callback: {done}/{self.max_progress_count}")
                    pass

    def _format_ros_timestamp(self, header):
        """
        Format ROS header timestamp as 'seconds_nanoseconds' with zero padding, or return 'no_timestamp' on error.
        """
        try:
            sec = header.stamp.sec
            nsec = header.stamp.nanosec
            return f"{sec:010d}_{nsec:09d}"
        except AttributeError:
            return "no_timestamp"