#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import argparse
import warnings
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main() -> None:
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    parser = argparse.ArgumentParser(
        description="Print the earliest source header timestamp for selected bag topics."
    )
    parser.add_argument("bag")
    parser.add_argument("topics", nargs="+")
    parser.add_argument("--index-dir", type=Path,
                        help="Write source timestamp to bag ordinal indexes for each topic")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = set(args.topics) - topic_types.keys()
    if missing:
        parser.error(f"topics absent from bag: {', '.join(sorted(missing))}")

    wanted = set(args.topics)
    source_times = {}
    indexes = {topic: [] for topic in args.topics}
    while reader.has_next() and (wanted or args.index_dir):
        topic, data, _ = reader.read_next()
        if topic not in indexes or (not args.index_dir and topic not in wanted):
            continue
        message = deserialize_message(data, get_message(topic_types[topic]))
        if not hasattr(message, "header"):
            parser.error(f"topic has no header timestamp: {topic}")
        stamp = message.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if source_ns <= 0:
            parser.error(f"topic has invalid first header timestamp: {topic}")
        if indexes[topic] and source_ns <= indexes[topic][-1]:
            parser.error(f"source timestamps must be strictly increasing for {topic}")
        indexes[topic].append(source_ns)
        source_times.setdefault(topic, source_ns)
        wanted.discard(topic)

    if wanted:
        parser.error(f"topics contain no messages: {', '.join(sorted(wanted))}")
    if args.index_dir:
        args.index_dir.mkdir(parents=True, exist_ok=True)
        for position, topic in enumerate(args.topics):
            (args.index_dir / f"topic-{position}.tsv").write_text("".join(
                f"{stamp}\t{ordinal}\n"
                for ordinal, stamp in enumerate(indexes[topic], 1)))
    print(min(source_times.values()))


if __name__ == "__main__":
    main()
