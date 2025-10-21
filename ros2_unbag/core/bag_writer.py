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

"""Writer utilities for creating ROS 2 bag files."""

from __future__ import annotations

from bisect import bisect_right
from collections import defaultdict
from pathlib import Path
from typing import Any, DefaultDict, Dict, Iterable, Mapping, MutableMapping, Optional, Sequence, Tuple, Union

from rosbags.highlevel import AnyWriter
from rosbags.typesys import Stores, get_typestore

BagPath = Union[str, Path]
TopicName = str
Timestamp = int
MessageEntry = Tuple[Timestamp, Any]
MessagesByTopic = Mapping[TopicName, Sequence[MessageEntry]]
TopicTypeMap = Mapping[TopicName, str]


class BagWriter:
    """Create ROS 2 bag files compatible with ``rosbags``."""

    def __init__(self, output_bag_path: BagPath, *, storage_id: str = "mcap") -> None:
        """Prepare the writer for the given output bag path.

        Args:
            output_bag_path: Target directory or file for the resulting bag.
            storage_id: Optional storage backend identifier understood by ``rosbags``.

        Returns:
            None
        """
        self._bag_path = Path(output_bag_path)
        self._storage_id = storage_id

        self._writer: Optional[AnyWriter] = None
        self._connections: MutableMapping[TopicName, Any] = {}
        self._connection_types: Dict[TopicName, str] = {}
        self._typestore = get_typestore(Stores.ROS2_JAZZY)

    def __enter__(self) -> "BagWriter":
        """Support context manager usage.

        Args:
            None

        Returns:
            BagWriter: This instance.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Ensure resources are released when leaving a context.

        Args:
            exc_type: Exception type if an error occurred.
            exc_val: Exception value if an error occurred.
            exc_tb: Exception traceback if an error occurred.

        Returns:
            None
        """
        self.close()

    @property
    def is_open(self) -> bool:
        """Return ``True`` when the underlying writer is open.

        Args:
            None

        Returns:
            bool: ``True`` if the writer has been opened.
        """
        return self._writer is not None

    def open(self, topic_types: TopicTypeMap, *, storage_id: Optional[str] = None) -> None:
        """Open the bag and register the provided topics.

        Args:
            topic_types: Mapping of topic names to ROS 2 message type strings.
            storage_id: Optional storage backend override. Defaults to the constructor value.

        Returns:
            None
        """
        if self.is_open:
            self.close()

        storage = storage_id or self._storage_id
        try:
            self._writer = AnyWriter(self._bag_path, storage_id=storage, default_typestore=self._typestore)
        except TypeError:
            # Fallback for older rosbags versions without default_typestore argument.
            self._writer = AnyWriter(self._bag_path, storage_id=storage)
        self._writer.open()

        self._connections.clear()
        self._connection_types = dict(topic_types)
        for topic, msg_type in topic_types.items():
            try:
                self._connections[topic] = self._writer.add_connection(topic, msg_type, typestore=self._typestore)
            except TypeError:
                self._connections[topic] = self._writer.add_connection(topic, msg_type)

    def close(self) -> None:
        """Close the bag writer and reset internal state.

        Args:
            None

        Returns:
            None
        """
        if self._writer is not None:
            self._writer.close()
            self._writer = None
        self._connections.clear()
        self._connection_types.clear()

    def write(self, topic: TopicName, message: Any, timestamp: Timestamp) -> None:
        """Serialize and write a single message to the bag.

        Args:
            topic: Name of the topic to write to.
            message: ROS 2 message instance ready for serialization.
            timestamp: Nanosecond timestamp associated with the message.

        Returns:
            None

        Raises:
            RuntimeError: If the writer has not been opened.
            ValueError: If the topic has not been registered.
        """
        if not self.is_open:
            raise RuntimeError("BagWriter must be opened before writing messages.")
        if topic not in self._connections:
            raise ValueError(f"Topic '{topic}' has not been registered; call open() first.")

        connection = self._connections[topic]
        msg_type = self._connection_types[topic]

        try:
            serialized = self._typestore.serialize_cdr(message, msg_type)
        except Exception as exc:
            raise TypeError(
                f"Failed to serialize message for topic '{topic}' of type '{msg_type}'. "
                "Ensure the payload matches the typestore definition or register the type explicitly."
            ) from exc

        self._writer.write(connection, timestamp, serialized)

    def write_synchronized(self, messages_by_topic: MessagesByTopic, reference_topic: TopicName) -> None:
        """Synchronize messages to the reference topic’s timeline and write them.

        For each timestamp emitted on the reference topic, the most recent message
        with a timestamp less than or equal to that reference time is selected for every
        other topic. When a topic has no samples prior to the reference timestamp the
        earliest available sample is used.

        Args:
            messages_by_topic: Mapping from topic name to sequences of ``(timestamp, message)`` tuples.
            reference_topic: Name of the topic providing the reference timeline.

        Returns:
            None

        Raises:
            ValueError: If the reference topic is missing from the provided data.
        """
        if reference_topic not in messages_by_topic:
            raise ValueError(f"Reference topic '{reference_topic}' is missing from message data.")

        sorted_messages = self._prepare_sorted(messages_by_topic)
        ref_timestamps, ref_entries = sorted_messages[reference_topic]

        for index, ref_time in enumerate(ref_timestamps):
            for topic, (timestamps, entries) in sorted_messages.items():
                if topic == reference_topic:
                    _, message = ref_entries[index]
                else:
                    choice = self._select_entry(timestamps, entries, ref_time)
                    _, message = choice
                self.write(topic, message, ref_time)

    def resample_and_write(
        self,
        reader: Any,
        selected_topics: Iterable[TopicName],
        reference_topic: Optional[TopicName],
    ) -> None:
        """Read, optionally synchronize, and persist messages from a :class:`BagReader`.

        Args:
            reader: Instance providing a ``read_messages`` generator and ``topic_types`` mapping.
            selected_topics: Topics to export to the new bag.
            reference_topic: Optional name of the topic used for synchronization.

        Returns:
            None
        """
        messages: DefaultDict[TopicName, list] = defaultdict(list)
        for topic, message, timestamp in reader.read_messages(selected_topics):
            messages[topic].append((timestamp, message))

        topic_types = {topic: reader.topic_types[topic] for topic in selected_topics}
        self.open(topic_types)

        if reference_topic is None:
            self._write_all(messages)
        else:
            self.write_synchronized(messages, reference_topic)

    def _prepare_sorted(
        self,
        messages_by_topic: MessagesByTopic,
    ) -> Dict[TopicName, Tuple[Sequence[Timestamp], Sequence[MessageEntry]]]:
        """Return sorted timestamp/message pairs per topic.

        Args:
            messages_by_topic: Mapping of topics to timestamped message sequences.

        Returns:
            Dict[str, Tuple[Sequence[int], Sequence[Tuple[int, Any]]]]: Sorted timestamps and entries.

        Raises:
            ValueError: If a topic contains no messages.
        """
        sorted_data: Dict[TopicName, Tuple[Sequence[Timestamp], Sequence[MessageEntry]]] = {}
        for topic, entries in messages_by_topic.items():
            if not entries:
                raise ValueError(f"Topic '{topic}' does not contain any messages.")
            ordered = sorted(entries, key=lambda item: item[0])
            timestamps = tuple(ts for ts, _ in ordered)
            sorted_data[topic] = (timestamps, ordered)
        return sorted_data

    @staticmethod
    def _select_entry(
        timestamps: Sequence[Timestamp],
        entries: Sequence[MessageEntry],
        target_time: Timestamp,
    ) -> MessageEntry:
        """Return the entry closest to ``target_time`` without exceeding it.

        Args:
            timestamps: Ordered timestamps for a topic.
            entries: Ordered message entries paired with ``timestamps``.
            target_time: Reference time to match against.

        Returns:
            Tuple[int, Any]: Selected timestamp and message pair.
        """
        index = bisect_right(timestamps, target_time) - 1
        if index < 0:
            index = 0
        return entries[index]

    def _write_all(self, messages_by_topic: Mapping[TopicName, Sequence[MessageEntry]]) -> None:
        """Write all messages in chronological order per topic without synchronization.

        Args:
            messages_by_topic: Mapping of topics to timestamped message sequences.

        Returns:
            None
        """
        for topic, entries in messages_by_topic.items():
            for timestamp, message in sorted(entries, key=lambda item: item[0]):
                self.write(topic, message, timestamp)
