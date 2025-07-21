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

import bisect
from collections import defaultdict

from rclpy.serialization import serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
)


class BagWriter:
    # Handles writing messages to a ROS2 bag file

    def __init__(self, output_bag_path):
        """
        Initialize BagWriter with output path and prepare SequentialWriter.

        Args:
            output_bag_path: Path to the output ROS2 bag file.

        Returns:
            None
        """
        self.output_bag_path = output_bag_path
        self.writer = SequentialWriter()

    def open(self, topic_types):
        """
        Configure and open the bag at output path, creating topics with given types.

        Args:
            topic_types: Dict mapping topic names to message type strings.

        Returns:
            None
        """
        storage_options = StorageOptions(uri=self.output_bag_path,
                                         storage_id='mcap')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                             output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)
        for topic, msg_type_str in topic_types.items():
            metadata = TopicMetadata(
                0,  # id
                topic,  # name
                msg_type_str,  # type
                'cdr',  # serialization_format
                [],  # offered_qos_profiles
                ''  # type_description_hash
            )
            self.writer.create_topic(metadata)

    def close(self):
        """
        Close the bag writer and release resources.

        Args:
            None

        Returns:
            None
        """
        del self.writer

    def write(self, topic, msg, timestamp):
        """
        Serialize and write a single message to the bag under the specified topic and timestamp.

        Args:
            topic: Topic name (str).
            msg: ROS2 message instance.
            timestamp: Timestamp for the message (int or float).

        Returns:
            None
        """
        # Write a single message to the bag
        self.writer.write(topic, serialize_message(msg), timestamp)

    def write_synchronized(self, messages_by_topic, reference_topic):
        """
        For each timestamp of the reference topic, select and write the nearest message (≤ timestamp) from each topic.

        Args:
            messages_by_topic: Dict mapping topic names to lists of (timestamp, message) tuples.
            reference_topic: Topic name to synchronize against (str).

        Returns:
            None
        """
        # Sort reference topic messages
        ref_msgs = sorted(messages_by_topic[reference_topic],
                          key=lambda x: x[0])
        ref_timestamps = [ts for ts, _ in ref_msgs]

        # Sort messages for each topic
        topic_ts_msg = {}
        for topic, msgs in messages_by_topic.items():
            sorted_msgs = sorted(msgs, key=lambda x: x[0])
            timestamps = [ts for ts, _ in sorted_msgs]
            topic_ts_msg[topic] = (timestamps, sorted_msgs)

        # For each reference timestamp, find the nearest (<=) message for each topic
        for i, t_sync in enumerate(ref_timestamps):
            for topic in messages_by_topic:
                timestamps, msgs = topic_ts_msg[topic]

                if topic == reference_topic:
                    msg = msgs[i][1]
                else:
                    idx = bisect.bisect_right(timestamps, t_sync) - 1
                    if idx < 0:
                        idx = 0
                    msg = msgs[idx][1]

                self.write(topic, msg, t_sync)

    def resample_and_write(self, reader, selected_topics, reference_topic):
        """
        Read messages for selected topics, open the bag, and write either all messages in order or synchronized to a reference topic.

        Args:
            reader: BagReader instance.
            selected_topics: List of topic names to export.
            reference_topic: Topic name to synchronize against, or None.

        Returns:
            None
        """
        messages_by_topic = defaultdict(list)
        for topic, msg, t in reader.read_messages(selected_topics):
            messages_by_topic[topic].append((t, msg))

        # Open bag for writing with selected topics
        self.open(
            {topic: reader.topic_types[topic] for topic in selected_topics})

        # Write all messages (optionally synchronized)
        if reference_topic is None:
            for topic, msgs in messages_by_topic.items():
                for t, msg in sorted(msgs, key=lambda x: x[0]):
                    self.write(topic, msg, t)
        else:
            self.write_synchronized(messages_by_topic, reference_topic)
