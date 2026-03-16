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
import logging
import multiprocessing as mp
from pathlib import Path
import threading
import traceback

from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata
from ros2_unbag.core.utils.file_utils import get_time_from_msg, substitute_placeholders, is_strftime_in_template


class Exporter:
    # Handles parallel export of messages from a ROS2 bag
    _MAX_DETAILED_FAILURE_LOGS = 5

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

        self.bag_reader = bag_reader
        self.config = export_config
        self.global_config = global_config
        self.topic_types = self.bag_reader.topic_types
        self.progress_callback = progress_callback
        self.continue_on_error = bool(self.global_config.get("continue_on_error", False))
        self.failed_items = mp.Value("i", 0)
        self.failure_counts = defaultdict(int)
        self._detailed_failure_logs_emitted = 0
        self._remaining_detailed_failures = mp.Value("i", self._MAX_DETAILED_FAILURE_LOGS)
        self.failure_queue = mp.Queue() if self.continue_on_error else None

        # Create a queue for worker exceptions to communicate back to the main process
        self.exception_queue = mp.Queue()

        # check if topic exists in the bag, if not, raise an error and list all available topics
        for topic in self.config:
            if topic not in self.topic_types:
                raise ValueError(f"Topic '{topic}' not found in bag. Available topics: {list(self.topic_types.keys())}")

        self.index_map = {t: 0 for t in self.config}
        self._resolved_formats = {}
        self._topic_cache = {}
        for topic, cfg in self.config.items():
            fmt = cfg['format']
            topic_type = self.topic_types[topic]
            resolution = ExportRoutine.resolve(topic_type, fmt)
            if resolution is None:
                raise ValueError(f"No export handler found for topic '{topic}' with format '{fmt}'")
            self._resolved_formats[topic] = resolution

        self.export_mode = {t: resolved[2] for t, resolved in self._resolved_formats.items()}
        self.sequential_topics = [t for t, m in self.export_mode.items() if m == ExportMode.SINGLE_FILE]

        # Default queue size to control memory usage (backpressure) and prevent OOM
        queue_size = max(1, int(self.global_config.get("queue_size", 500)))
        
        # one queue for parallel topics
        self.parallel_q = mp.Queue(maxsize=queue_size)
        # one queue per sequential topic
        self.seq_queues = {t: mp.Queue(maxsize=queue_size) for t in self.sequential_topics}

        cpu_percentage = float(self.global_config.get("cpu_percentage", 80.0))
        cpu_percentage = max(0.0, min(100.0, cpu_percentage))
        self.num_workers = max(1, int(mp.cpu_count() * cpu_percentage * 0.01))
        self.num_parallel_workers = max(1, self.num_workers - len(self.sequential_topics))

        self.logger.info(
            "Using %d workers for export, %d for parallel topics, %d for sequential topics.",
            self.num_workers,
            self.num_parallel_workers,
            len(self.sequential_topics),
        )
        self._enqueued_files = set()

        # Pre-fetch export handlers and processors
        self.topic_handlers = {}
        self.topic_processors = {}

        for topic, cfg in self.config.items():

            fmt = cfg['format']
            topic_type = self.topic_types[topic]
            routine, canonical_fmt, mode = self._resolved_formats[topic]

            # Export handler
            handler = routine.func
            if handler is None:
                raise ValueError(f"No export handler found for topic '{topic}' with format '{fmt}'")
            self.topic_handlers[topic] = handler

            # Optional processor chain
            processor_chain = self._prepare_processor_chain(topic, cfg, topic_type)
            if processor_chain:
                self.topic_processors[topic] = processor_chain
            else:
                self.topic_processors[topic] = []

            # Prepare naming and path
            name_tmpl = cfg['naming']
            path_tmpl = cfg['path']
            sub_tmpl  = cfg.get('subfolder', '').strip('/')

            uses_index_or_ts = any(x in s for s in (name_tmpl, path_tmpl, sub_tmpl)
                                for x in ("%index", "%timestamp", "%master_timestamp"))
            has_strftime_name = is_strftime_in_template(name_tmpl)
            has_strftime_path = (is_strftime_in_template(path_tmpl) or is_strftime_in_template(sub_tmpl))

            # Check for conflicting templates
            if self.export_mode[topic] == ExportMode.SINGLE_FILE and (uses_index_or_ts or has_strftime_name or has_strftime_path):
                raise ValueError(
                    f"You are using a routine that stores data to one single file for topic: {topic}. You can not use %index/%timestamp/%master_timestamp and strftime in the naming, path or subfolder. This would lead to multiple files being created."
                )
            if self.export_mode[topic] == ExportMode.MULTI_FILE and not (uses_index_or_ts or has_strftime_name):
                raise ValueError(
                    f"You are using a routine that stores data to multiple files for topic: {topic}. This requires %index/%timestamp/%master_timestamp in naming/path/subfolder. Else data would be overwritten."
                )

            # Cache per-topic data
            if not hasattr(self, "_topic_cache"):
                self._topic_cache = {}
            self._topic_cache[topic] = {
                "fmt": canonical_fmt,
                "mode": mode,
                "sequential": (mode == ExportMode.SINGLE_FILE),
                "topic_base": topic.strip("/").replace("/", "_"),
                "name_tmpl": name_tmpl,
                "path_tmpl": path_tmpl,
                "sub_tmpl":  sub_tmpl,
                "has_strftime_name": has_strftime_name,
                "has_strftime_path": has_strftime_path,
            }

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

        # Max index for each topic
        self.max_index = {key: count - 1 for key, count in self.message_count.items()}
        self.index_length = {key: max(1, len(str(count - 1))) for key, count in self.message_count.items()}

        # Queue for progress
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
        failure_monitor = None
        if self.continue_on_error:
            failure_monitor = threading.Thread(
                target=self._monitor_failures,
                name="FailureMonitor",
                daemon=True,
            )
            failure_monitor.start()

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

                    err_msg = "Export exited with error"
                    if (len(workers) > 1):
                        err_msg = f"{err_msg} (export results may be incomplete due to parallel processing)"

                    raise RuntimeError(f"{err_msg}: [{exc_type}] {exc_msg}")

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
        finally:
            progress_queue.put(None)
            monitor.join()
            if failure_monitor is not None:
                self.failure_queue.put(None)
                failure_monitor.join()

    def abort_export(self):
        """
        Abort export by throwing a user abort exception.

        Args:
            None

        Returns:
            None
        """
        error = RuntimeError("Export aborted by user")
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
            self.logger.info(
                "Resampling with strategy '%s' to master topic '%s'",
                assoc,
                master,
            )
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
        discard_eps_ns = int(discard_eps * 1e9) if discard_eps is not None else None

        while True:
            res = self.bag_reader.read_next_message()
            if res is None:
                break

            topic, msg, _ = res
            if topic not in self.config:
                continue

            ts = get_time_from_msg(msg, return_datetime=False)

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
                if discard_eps_ns is not None and abs(master_ts - sel_ts) > discard_eps_ns:
                    frame = None
                    break
                frame[t] = sel_msg

            if frame:
                for t, m in frame.items():
                    self._enqueue_export_task(t, m, master_ts=master_ts)
            else:
                for t in self.config:
                    if t == master_topic:
                        continue
                    if t not in latest_messages or (
                        discard_eps_ns is not None and abs(master_ts - latest_messages[t][0]) > discard_eps_ns
                    ):
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
        latest_ts_seen = 0
        discard_eps_ns = int(discard_eps * 1e9)

        while True:
            res = self.bag_reader.read_next_message()
            if res is None:
                break

            topic, msg, _ = res
            if topic not in self.config:
                continue

            ts = get_time_from_msg(msg, return_datetime=False)
            
            latest_ts_seen = max(latest_ts_seen, ts)
            buffers[topic].append((ts, msg))

            # Attempt to process buffered master messages with the data seen so far
            self._drain_nearest_frames(
                master_topic=master_topic,
                buffers=buffers,
                latest_ts_seen=latest_ts_seen,
                discard_eps_ns=discard_eps_ns,
                dropped_frames=dropped_frames,
                flush=False,
            )

            # Remove stale messages from buffers
            expire_before = latest_ts_seen - discard_eps_ns * 2
            for t in buffers:
                while buffers[t] and buffers[t][0][0] < expire_before:
                    buffers[t].popleft()

        # Final drain in case the last master frames waited for lookahead data
        self._drain_nearest_frames(
            master_topic=master_topic,
            buffers=buffers,
            latest_ts_seen=latest_ts_seen,
            discard_eps_ns=discard_eps_ns,
            dropped_frames=dropped_frames,
            flush=True,
        )

    def _drain_nearest_frames(self, master_topic, buffers, latest_ts_seen,
                              discard_eps_ns, dropped_frames, flush):
        """
        Drain buffered master frames and attempt to assemble synchronized frames using nearest association.

        Try to process buffered master frames. When flush is False, wait until the
        buffer has seen data beyond (master_ts + discard_eps_ns) to avoid
        premature drops. On final flush, process whatever is available.

        Args:
            master_topic (str): Name of the master topic.
            buffers (defaultdict(deque)): Per-topic buffers containing (timestamp, msg) tuples.
            latest_ts_seen (int): Latest timestamp observed across topics (nanoseconds).
            discard_eps_ns (int): Discard epsilon threshold in nanoseconds.
            dropped_frames (dict): Mapping of topic -> dropped frame counts to update.
            flush (bool): If True, force processing of remaining master frames.

        Returns:
            None
        """
        while buffers[master_topic]:
            candidate_ts, candidate_msg = buffers[master_topic][0]
            if not flush and candidate_ts + discard_eps_ns > latest_ts_seen:
                break  # Wait for more data to safely evaluate this master frame

            master_ts = candidate_ts
            frame = {master_topic: candidate_msg}
            valid = True

            # Find best match from each topic
            for t in self.config:
                if t == master_topic:
                    continue
                candidates = [
                    (ts_, msg_)
                    for ts_, msg_ in buffers[t]
                    if abs(ts_ - master_ts) <= discard_eps_ns
                ]
                if not candidates:
                    valid = False
                    break
                selected_ts, selected_msg = min(
                    candidates, key=lambda x: abs(x[0] - master_ts))
                frame[t] = selected_msg

            if valid:
                for t, m in frame.items():
                    self._enqueue_export_task(t, m, master_ts=master_ts)
            else:
                for t in self.config:
                    if t == master_topic:
                        continue
                    if not any(
                        abs(ts_ - master_ts) <= discard_eps_ns for ts_, _ in buffers[t]):
                        dropped_frames[t] += 1

            # Remove processed master message
            buffers[master_topic].popleft()

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
        self.logger.info(
            "The synchronization process dropped frames caused by the discard eps.\n"
            "The following topics were not available at frame generation time:"
        )
        for topic, count in dropped_frames.items():
            self.logger.info("  %s: %d times", topic, count)


    def _enqueue_export_task(self, topic, msg, master_ts=None):
        """
        Build filename and directory for a topic message, create path, and enqueue the export task with format.

        Args:
            topic: Topic name (str).
            msg: ROS2 message instance.
            master_ts: Optional timestamp of the master topic (float/int).

        Returns:
            None
        """
        # Fetch per-topic cache
        cache = self._topic_cache[topic]

        # Handle indexing
        index = self.index_map[topic]
        self.index_map[topic] += 1

        # Apply naming pattern
        idx_len = self.index_length[topic]
        ts_float = get_time_from_msg(msg, return_datetime=False)
        replacements = {
            "name": cache["topic_base"],
            "index": str(index).zfill(idx_len),
            "timestamp": str(ts_float),
            "master_timestamp": str(master_ts) if master_ts is not None else str(ts_float),
        }

        naming = substitute_placeholders(cache["name_tmpl"], replacements)
        path = substitute_placeholders(cache["path_tmpl"], replacements)
        subfolder = substitute_placeholders(cache["sub_tmpl"], replacements)

        # Strftime only applies if the naming contains strftime directives
        if cache["has_strftime_name"]:
            timestamp = get_time_from_msg(msg, return_datetime=True)
            filename = timestamp.strftime(naming)
        else:
            filename = naming

        path = Path(path) / subfolder
        full_path = path / filename

        # Determine if this is the first time this file is being enqueued
        is_first = full_path not in self._enqueued_files

        # Abort if the file name does not change in MULTI_FILE mode
        if cache["mode"] == ExportMode.MULTI_FILE and not is_first:
            raise ValueError(f"Cannot use a non-changing file name for topic '{topic}' "
                             f"and format '{cache['fmt']}'. This will overwrite the previous file: {full_path}")

        # Create the directory if it does not exist
        path.mkdir(parents=True, exist_ok=True)

        # Enqueue the export task
        self._enqueued_files.add(full_path)

        # Create metadata for the export
        metadata = ExportMetadata(index=index, max_index=self.max_index[topic])

        task = (topic, msg, full_path, cache["fmt"], metadata)
        if cache["sequential"]:
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
                processor_chain = self.topic_processors.get(topic) or []
                for handler, args in processor_chain:
                    msg = handler(msg=msg, **args)

                # Use pre-fetched export handler
                export_handler = self.topic_handlers[topic]
                if export_handler:
                    export_handler(msg, full_path, fmt, metadata, topic=topic)
                    progress_queue.put(1)

            except Exception as e:
                # Handle exceptions during export
                if self.continue_on_error:
                    with self.failed_items.get_lock():
                        self.failed_items.value += 1
                    include_traceback = False
                    with self._remaining_detailed_failures.get_lock():
                        if self._remaining_detailed_failures.value > 0:
                            self._remaining_detailed_failures.value -= 1
                            include_traceback = True
                    self.failure_queue.put((
                        topic if "topic" in locals() else "<unknown>",
                        str(full_path) if "full_path" in locals() else "<unknown>",
                        type(e).__name__,
                        str(e),
                        traceback.format_exc() if include_traceback else None,
                    ))
                    progress_queue.put(1)
                    continue

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
                    self.logger.exception(
                        "Error in progress callback at %d/%d",
                        done,
                        self.max_progress_count,
                    )

    def _monitor_failures(self):
        """
        Aggregate worker-level processing/export failures and emit bounded detail logs.

        Args:
            None

        Returns:
            None
        """
        while True:
            event = self.failure_queue.get()
            if event is None:
                break

            topic, output_path, exc_type, exc_msg, exc_traceback = event
            signature = (topic, exc_type, exc_msg)
            self.failure_counts[signature] += 1

            if self._detailed_failure_logs_emitted >= self._MAX_DETAILED_FAILURE_LOGS:
                continue

            if exc_traceback is None:
                continue

            self._detailed_failure_logs_emitted += 1
            self.logger.error(
                "Skipping failed item for topic '%s' (output: %s) due to %s: %s\n%s",
                topic,
                output_path,
                exc_type,
                exc_msg,
                exc_traceback.rstrip(),
            )

        if not self.failure_counts:
            return

        suppressed_count = self.failed_item_count - self._detailed_failure_logs_emitted
        if suppressed_count > 0:
            self.logger.warning(
                "Suppressed detailed logs for %d additional failed item(s).",
                suppressed_count,
            )

        self.logger.warning(
            "Skipped %d item(s) due to processing/export errors. Failure summary:",
            self.failed_item_count,
        )
        for (topic, exc_type, exc_msg), count in sorted(
            self.failure_counts.items(),
            key=lambda item: (-item[1], item[0][0], item[0][1], item[0][2]),
        ):
            self.logger.warning(
                "  %s x%d on topic '%s': %s",
                exc_type,
                count,
                topic,
                exc_msg,
            )

    def _prepare_processor_chain(self, topic, cfg, topic_type):
        """
        Normalize processor configuration into an ordered list of (handler, args) tuples for a topic.

        Args:
            topic: Topic name (str).
            cfg: Topic configuration dictionary.
            topic_type: ROS message type string.

        Returns:
            list: Ordered list of (handler, args_dict) tuples.
        """
        raw_chain = cfg.get("processors")
        if raw_chain is None and "processor" in cfg:
            raw_chain = [{
                "name": cfg["processor"],
                "args": cfg.get("processor_args", {}),
            }]

        if raw_chain is None:
            return []

        if not isinstance(raw_chain, list):
            raise ValueError(
                f"Processors for topic '{topic}' must be provided as a list."
            )

        normalized = []
        canonical_chain = []

        for idx, entry in enumerate(raw_chain, start=1):
            if isinstance(entry, str):
                proc_name = entry
                proc_args = {}
            elif isinstance(entry, dict):
                if "name" not in entry:
                    raise ValueError(
                        f"Processor entry #{idx} for topic '{topic}' is missing the 'name' field."
                    )
                proc_name = entry["name"]
                proc_args = entry.get("args", {})
                if proc_args is None:
                    proc_args = {}
                if not isinstance(proc_args, dict):
                    raise ValueError(
                        f"'args' for processor '{proc_name}' on topic '{topic}' must be a dictionary."
                    )
            else:
                raise ValueError(
                    f"Processor entry #{idx} for topic '{topic}' must be a string or dict."
                )

            proc_handler = Processor.get_handler(topic_type, proc_name)
            if proc_handler is None:
                raise ValueError(
                    f"No processor handler found for topic '{topic}' with processor '{proc_name}'."
                )

            required_args = Processor.get_required_args(topic_type, proc_name)
            missing_args = [arg for arg in required_args if arg not in proc_args]
            if missing_args:
                raise ValueError(
                    f"Missing required arguments for processor '{proc_name}' on topic '{topic}': {', '.join(missing_args)}"
                )

            normalized.append((proc_handler, dict(proc_args)))
            canonical_chain.append({
                "name": proc_name,
                "args": dict(proc_args),
            })

        # Update configuration with canonical representation for downstream consumers
        cfg["processors"] = canonical_chain
        cfg.pop("processor", None)
        cfg.pop("processor_args", None)

        return normalized

    @property
    def failed_item_count(self) -> int:
        """
        Return the number of items skipped due to worker-level processing/export errors.

        Args:
            None

        Returns:
            int: Number of failed items encountered by workers.
        """
        return int(self.failed_items.value)
