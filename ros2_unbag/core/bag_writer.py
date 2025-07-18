import bisect
from collections import defaultdict

from rclpy.serialization import serialize_message
from rosbag2_py import (ConverterOptions, SequentialWriter, StorageOptions,
                        TopicMetadata)


class BagWriter:
    # Handles writing messages to a ROS2 bag file (MCAP format)

    def __init__(self, output_bag_path):
        self.output_bag_path = output_bag_path
        self.writer = SequentialWriter()

    def open(self, topic_types):
        # Initialize bag writer with given topic types
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
        # Close writer
        del self.writer

    def write(self, topic, msg, timestamp):
        # Write a single message to the bag
        self.writer.write(topic, serialize_message(msg), timestamp)

    def write_synchronized(self, messages_by_topic, reference_topic):
        # Write messages synchronized to reference topic timestamps

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
        # Collect messages and write them (optionally synchronized)

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
