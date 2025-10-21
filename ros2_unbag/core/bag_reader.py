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

"""High-level helpers for accessing ROS 2 bag data."""

from __future__ import annotations

from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Deque, DefaultDict, Dict, Iterable, Iterator, Optional, Sequence, Tuple, Union

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

BagPath = Union[str, Path]
TopicName = str
Timestamp = int
MessageTuple = Tuple[TopicName, Any, Timestamp]


class BagReader:
    """Read messages and metadata from a ROS 2 bag."""

    def __init__(self, bag_path: BagPath) -> None:
        """Open the given bag and populate topic metadata.

        Args:
            bag_path: Path to a ROS 2 bag directory or database file.

        Returns:
            None
        """
        self._bag_path = Path(bag_path)
        self.reader: Optional[AnyReader] = None
        self.topic_types: Dict[TopicName, str] = {}
        self.metadata: Any = None

        self._tf_queue: Deque[MessageTuple] = deque()
        self._topic_connections: DefaultDict[TopicName, list] = defaultdict(list)
        self._all_connections: Tuple[Any, ...] = tuple()
        self._selected_topics: Optional[Sequence[TopicName]] = None

        self._message_iter: Optional[Iterator[Tuple[Any, Timestamp, bytes]]] = None
        self._next_record: Optional[Tuple[Any, Timestamp, bytes]] = None
        self._message_counts: Optional[Dict[TopicName, int]] = None
        self._typestore = get_typestore(Stores.ROS2_JAZZY)

        self._open()

    def __enter__(self) -> "BagReader":
        """Support use as a context manager.

        Args:
            None

        Returns:
            BagReader: This instance.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Close the underlying reader when leaving a context.

        Args:
            exc_type: Exception type if an error occurred.
            exc_val: Exception value if an error occurred.
            exc_tb: Exception traceback if an error occurred.

        Returns:
            None
        """
        self.close()

    def close(self) -> None:
        """Release resources associated with the reader.

        Args:
            None

        Returns:
            None
        """
        if self.reader is not None:
            self.reader.close()
            self.reader = None

    def _open(self) -> None:
        """Open the bag and index topic connections.

        Args:
            None

        Returns:
            None

        Raises:
            RuntimeError: If the bag cannot be opened.
        """
        path = self._resolve_bag_path()
        try:
            self.reader = AnyReader([path], default_typestore=self._typestore)
            self.reader.open()
        except Exception as exc:
            raise RuntimeError(f"Failed to open bag at '{path}': {exc}") from exc

        self._index_connections()
        self.metadata = getattr(self.reader, "metadata", None)
        self._message_counts = self._extract_message_counts()
        self._reset_iterator()

    def _resolve_bag_path(self) -> Path:
        """Resolve the user-provided path to a bag directory.

        Args:
            None

        Returns:
            Path: Resolved directory or file path to use with the reader.

        Raises:
            FileNotFoundError: If the supplied path does not exist.
        """
        path = self._bag_path.expanduser()
        if path.is_file():
            if path.suffix.lower() == ".db3":
                parent = path.parent
                if (parent / "metadata.yaml").exists():
                    return parent
            return path
        if path.is_dir():
            return path
        raise FileNotFoundError(f"Bag path does not exist: {path}")

    def _index_connections(self) -> None:
        """Populate topic maps from the active reader.

        Args:
            None

        Returns:
            None

        Raises:
            RuntimeError: If the reader has not been opened yet.
        """
        if self.reader is None:
            raise RuntimeError("Reader not opened before indexing connections.")

        self.topic_types.clear()
        self._topic_connections.clear()

        self._all_connections = tuple(self.reader.connections)
        for connection in self._all_connections:
            self.topic_types.setdefault(connection.topic, connection.msgtype)
            self._topic_connections[connection.topic].append(connection)

    def _extract_message_counts(self) -> Optional[Dict[TopicName, int]]:
        """Read per-topic message counts from connection metadata.

        Args:
            None

        Returns:
            Optional[Dict[str, int]]: Mapping of topics to stored message counts, if available.
        """
        counts: DefaultDict[TopicName, int] = defaultdict(int)
        for connection in self._all_connections:
            msgcount = getattr(connection, "msgcount", None)
            if msgcount is not None:
                counts[connection.topic] += msgcount
        return dict(counts) if counts else None

    def _get_connections_for_topics(self, topics: Optional[Iterable[TopicName]]) -> Tuple[Any, ...]:
        """Return connection records matching the provided topics.

        Args:
            topics: Optional iterable of topic names to include.

        Returns:
            Tuple[Any, ...]: Matching connection instances.
        """
        if topics is None:
            return self._all_connections
        selected = []
        for topic in topics:
            selected.extend(self._topic_connections.get(topic, []))
        return tuple(selected)

    def _reset_iterator(self) -> None:
        """Reset the streaming iterator to honour the current topic filter.

        Args:
            None

        Returns:
            None

        Raises:
            RuntimeError: If the reader has not been opened yet.
        """
        if self.reader is None:
            raise RuntimeError("Reader not opened before resetting iterator.")
        connections = self._get_connections_for_topics(self._selected_topics)
        self._message_iter = self.reader.messages(connections=connections)
        self._next_record = None

    def _prefetch_next(self) -> None:
        """Fetch the next record from the iterator if not cached.

        Args:
            None

        Returns:
            None
        """
        if self._message_iter is None or self._next_record is not None:
            return
        try:
            self._next_record = next(self._message_iter)
        except StopIteration:
            self._next_record = None

    def _pop_next_record(self) -> Optional[Tuple[Any, Timestamp, bytes]]:
        """Return the next raw record from the reader.

        Args:
            None

        Returns:
            Optional[Tuple[Any, int, bytes]]: Next raw connection, timestamp, and data tuple.
        """
        self._prefetch_next()
        record = self._next_record
        self._next_record = None
        return record

    def _deserialize(self, connection: Any, data: bytes) -> Any:
        """Turn raw message bytes into a ROS message instance.

        Args:
            connection: Connection descriptor including the topic name.
            data: Serialized message payload.

        Returns:
            Any: Deserialized ROS message instance.
        """
        if self.reader is None:
            raise RuntimeError("Reader must be open before deserializing messages.")
        return self.reader.deserialize(data, connection.msgtype)

    def get_topics(self) -> Dict[str, list]:
        """Return a mapping of message type to the list of topics providing it.

        Args:
            None

        Returns:
            Dict[str, list]: Mapping from message type string to topic names.
        """
        topics: DefaultDict[str, list] = defaultdict(list)
        for topic, msg_type in self.topic_types.items():
            topics[msg_type].append(topic)
        return dict(topics)

    def get_message_count(self) -> Dict[TopicName, int]:
        """Return per-topic message counts collected from metadata or iteration.

        Args:
            None

        Returns:
            Dict[str, int]: Mapping from topic name to counted messages.

        Raises:
            RuntimeError: If the reader has not been opened yet.
        """
        if self._message_counts is None:
            if self.reader is None:
                raise RuntimeError("Reader must be open to collect message counts.")
            counts: DefaultDict[TopicName, int] = defaultdict(int)
            for connection, _, _ in self.reader.messages():
                counts[connection.topic] += 1
            self._message_counts = dict(counts)
        return self._message_counts

    def get_topics_with_frequency(self) -> list:
        """Return per-topic statistics including approximate publishing frequency.

        Args:
            None

        Returns:
            list: Sequence of dictionaries containing name, type, and frequency.

        Raises:
            RuntimeError: If the reader has not been opened yet.
        """
        if self.reader is None:
            raise RuntimeError("Reader must be open to compute frequencies.")

        topic_timestamps: DefaultDict[TopicName, list] = defaultdict(list)
        for connection, timestamp, _ in self.reader.messages():
            topic_timestamps[connection.topic].append(timestamp)

        topic_stats = []
        for topic, timestamps in topic_timestamps.items():
            timestamps.sort()
            if len(timestamps) > 1:
                duration = (timestamps[-1] - timestamps[0]) / 1e9
                frequency = len(timestamps) / duration if duration > 0 else 0.0
            else:
                frequency = 0.0
            topic_stats.append({
                "name": topic,
                "type": self.topic_types.get(topic, "unknown"),
                "frequency": frequency,
            })
        return topic_stats

    def set_filter(self, selected_topics: Optional[Iterable[TopicName]]) -> None:
        """Restrict subsequent streaming reads to the provided topic names.

        Args:
            selected_topics: Iterable of topic names to include, or ``None`` for all topics.

        Returns:
            None
        """
        self._selected_topics = None if selected_topics is None else tuple(selected_topics)
        self._reset_iterator()

    def read_next_message(self) -> Optional[MessageTuple]:
        """Return the next deserialized message respecting the active topic filter.

        TF messages are expanded so that each transform is yielded individually. When no
        further messages are available, ``None`` is returned.

        Args:
            None

        Returns:
            Optional[Tuple[str, Any, int]]: Deserialized message triple or ``None``.

        Raises:
            RuntimeError: If deserialization fails.
        """
        if self._tf_queue:
            return self._tf_queue.popleft()

        record = self._pop_next_record()
        if record is None:
            return None

        try:
            connection, timestamp, data = record
            message = self._deserialize(connection, data)
        except Exception as exc:
            raise RuntimeError(f"Failed to read message from topic '{connection.topic}': {exc}") from exc

        if connection.msgtype == "tf2_msgs/msg/TFMessage" or hasattr(message, "transforms"):
            for transform in message.transforms:
                self._tf_queue.append((connection.topic, transform, timestamp))
            return self._tf_queue.popleft() if self._tf_queue else None

        return connection.topic, message, timestamp

    def read_messages(self, selected_topics: Optional[Iterable[TopicName]] = None) -> Iterator[MessageTuple]:
        """Stream deserialized messages for the requested topics.

        Args:
            selected_topics: Optional iterable restricting the topics to include. Passing
                ``None`` streams every topic in the bag.

        Yields:
            Tuple[str, Any, int]: Deserialized message triple ordered by recorded time.

        Raises:
            RuntimeError: If the reader has not been opened or iteration fails.
        """
        if self.reader is None:
            raise RuntimeError("Reader must be open to iterate messages.")

        connections = self._get_connections_for_topics(selected_topics)
        try:
            for connection, timestamp, data in self.reader.messages(connections=connections):
                yield connection.topic, self._deserialize(connection, data), timestamp
        except Exception as exc:
            raise RuntimeError("Error while reading bag messages.") from exc
