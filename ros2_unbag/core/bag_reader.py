import os
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosbag2_py import (ConverterOptions, SequentialReader, StorageFilter,
                        StorageOptions)
from rosidl_runtime_py.utilities import get_message


class BagReader:
    # Reads messages and metadata from a ROS2 bag file

    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.reader = SequentialReader()
        self.topic_types = {}
        self.metadata = None
        self._open_bag()

    def _detect_storage_id(self):
        # Determine storage format based on file extension
        ext = os.path.splitext(self.bag_path)[1].lower()
        if ext == '.db3':
            return 'sqlite3'
        elif ext == '.mcap':
            return 'mcap'
        else:
            raise ValueError(f"Unsupported bag extension: {ext}")

    def _open_bag(self):
        # Open the bag file and extract topic types and metadata
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
        # Return topics grouped by message type
        topics = defaultdict(list)
        for topic, msg_type in self.topic_types.items():
            topics[msg_type].append(topic)
        return dict(topics)

    def get_message_count(self):
        # Return message count for each topic
        if not self.metadata:
            raise RuntimeError("Bag metadata not available.")
        return {
            topic.topic_metadata.name: topic.message_count
            for topic in self.metadata.topics_with_message_count
        }

    def get_topics_with_frequency(self):
        # Calculate approximate frequency for each topic
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
        # Set topic filter for reading messages
        self.reader.set_filter(StorageFilter(topics=list(selected_topics)))

    def read_next_message(self):
        # Read the next message from the bag
        if not self.reader.has_next():
            return None
        try:
            topic, data, t = self.reader.read_next()
            msg_type = get_message(self.topic_types[topic])
            msg = deserialize_message(data, msg_type)
            return topic, msg, t
        except Exception as e:
            raise RuntimeError(f"Failed to read message: {e}")

    def read_messages(self, selected_topics):
        # Generator to iterate over messages for selected topics
        self.set_filter(selected_topics)
        while self.reader.has_next():
            try:
                topic, data, t = self.reader.read_next()
                msg_type = get_message(self.topic_types[topic])
                msg = deserialize_message(data, msg_type)
                yield topic, msg, t
            except Exception as e:
                raise RuntimeError(f"Error while reading messages: {e}")
