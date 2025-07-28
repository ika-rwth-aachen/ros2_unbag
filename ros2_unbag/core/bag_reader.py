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

from collections import defaultdict
import os

from tf2_msgs.msg import TFMessage

from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageFilter,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message


class BagReader:
    # Reads messages and metadata from a ROS2 bag file

    def __init__(self, bag_path):
        """
        Initialize BagReader with bag path, open the bag, and load topic types and metadata.

        Args:
            bag_path: Path to the ROS2 bag file.

        Returns:
            None
        """
        self.bag_path = bag_path
        self.reader = SequentialReader()
        self.topic_types = {}
        self.metadata = None
        self._open_bag()

    def _detect_storage_id(self):
        """
        Determine storage format ('sqlite3' or 'mcap') from bag file extension.

        Args:
            None

        Returns:
            str: Storage format identifier.

        Raises:
            ValueError: If extension is unsupported.
        """
        ext = os.path.splitext(self.bag_path)[1].lower()
        if ext == '.db3':
            return 'sqlite3'
        elif ext == '.mcap':
            return 'mcap'
        else:
            raise ValueError(f"Unsupported bag extension: {ext}")

    def _open_bag(self):
        """
        Open the bag with appropriate storage and converter options, and populate topic types and metadata.

        Args:
            None

        Returns:
            None

        Raises:
            RuntimeError: If bag cannot be opened.
        """
        try:
            storage_id = self._detect_storage_id()
            storage_options = StorageOptions(uri=self.bag_path,
                                             storage_id=storage_id)
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr')
            self.reader.open(storage_options, converter_options)
            self.topic_types = {
                t.name: t.type for t in self.reader.get_all_topics_and_types()
            }
            self.metadata = self.reader.get_metadata()
        except Exception as e:
            raise RuntimeError(f"Failed to open bag: {e}")

    def get_topics(self):
        """
        Return a dict grouping topics by their message type.

        Args:
            None

        Returns:
            dict: Mapping of message type to list of topics.
        """
        topics = defaultdict(list)
        for topic, msg_type in self.topic_types.items():
            topics[msg_type].append(topic)
        return dict(topics)

    def get_message_count(self):
        """
        Return a dict of message counts per topic from the bag metadata.

        Args:
            None

        Returns:
            dict: Mapping of topic name to message count.

        Raises:
            RuntimeError: If metadata is not available.
        """
        if not self.metadata:
            raise RuntimeError("Bag metadata not available.")
        return {
            topic.topic_metadata.name: topic.message_count
            for topic in self.metadata.topics_with_message_count
        }

    def get_topics_with_frequency(self):
        """
        Calculate and return approximate frequency (messages per second) for each topic.

        Args:
            None

        Returns:
            list: List of dicts with topic name, type, and frequency.

        Raises:
            RuntimeError: If frequency calculation fails.
        """
        try:
            reader = SequentialReader()
            storage_id = self._detect_storage_id()
            storage_options = StorageOptions(uri=self.bag_path,
                                             storage_id=storage_id)
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr')
            reader.open(storage_options, converter_options)

            topic_timestamps = defaultdict(list)
            while reader.has_next():
                topic, _, t = reader.read_next()
                topic_timestamps[topic].append(t)

            result = []
            for topic, timestamps in topic_timestamps.items():
                timestamps.sort()
                duration = (timestamps[-1] -
                            timestamps[0]) / 1e9 if len(timestamps) > 1 else 0.0
                frequency = len(timestamps) / duration if duration > 0 else 0.0
                result.append({
                    "name": topic,
                    "type": self.topic_types.get(topic, "unknown"),
                    "frequency": frequency
                })

            return result
        except Exception as e:
            raise RuntimeError(f"Failed to calculate frequencies: {e}")

    def set_filter(self, selected_topics):
        """
        Apply a topic filter to the reader for subsequent message reads.

        Args:
            selected_topics: Iterable of topic names to filter.

        Returns:
            None
        """
        self.reader.set_filter(StorageFilter(topics=list(selected_topics)))

    def read_next_message(self):
        """
        Read, deserialize, and return the next message (topic, msg, timestamp), or None if done.

        Args:
            None

        Returns:
            tuple or None: (topic, msg, timestamp) or None if no more messages.

        Raises:
            RuntimeError: If reading or deserialization fails.
        """
        if not self.reader.has_next():
            return None
        try:
            topic, data, t = self.reader.read_next()
            msg_type = get_message(self.topic_types[topic])
            msg = deserialize_message(data, msg_type)

            # Handle TFMessage specifically to extract the first transform
            if type(msg) is TFMessage:
                if msg.transforms:
                    msg = msg.transforms[0]
                else:
                    return None
                
            return topic, msg, t
        except Exception as e:
            raise RuntimeError(f"Failed to read message: {e}")

    def read_messages(self, selected_topics):
        """
        Generator yielding deserialized messages and timestamps for selected topics.

        Args:
            selected_topics: Iterable of topic names to filter.

        Returns:
            generator: Yields (topic, msg, timestamp) tuples.

        Raises:
            RuntimeError: If reading or deserialization fails.
        """
        self.set_filter(selected_topics)
        while self.reader.has_next():
            try:
                topic, data, t = self.reader.read_next()
                msg_type = get_message(self.topic_types[topic])
                msg = deserialize_message(data, msg_type)
                yield topic, msg, t
            except Exception as e:
                raise RuntimeError(f"Error while reading messages: {e}")
