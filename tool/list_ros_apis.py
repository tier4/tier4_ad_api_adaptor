#!/usr/bin/env python3
import argparse

import rclpy


class RosNode:
    def __init__(self, node_name, namespace, wait_time):
        self.node_name = node_name
        self.namespace = namespace
        self.wait_time = wait_time
        self.node = None
        self.wait = True

    def __enter__(self):
        rclpy.init()
        self.node = rclpy.create_node(node_name=self.node_name, namespace=self.namespace)
        self.spin()
        return self

    def __exit__(self, except_type, except_value, traceback):
        rclpy.shutdown()

    def on_timer(self):
        self.wait = False

    def spin(self):
        timer = self.node.create_timer(self.wait_time, self.on_timer)
        while self.wait:
            rclpy.spin_once(self.node)
        self.node.destroy_timer(timer)

    def get_topic_names_and_types(self):
        return self.__remove_builtins(self.node.get_topic_names_and_types())

    def get_service_names_and_types(self):
        return self.__remove_builtins(self.node.get_service_names_and_types())

    @staticmethod
    def __remove_builtins(names_and_types):
        builtins = ["rcl_interfaces", "composition_interfaces"]
        for name, types in names_and_types:
            if len(types) == 1:
                for prefix in builtins:
                    if name.startswith(prefix):
                        yield name, types


def is_external_api(name):
    return (
        name.startswith("/api/external/")
        or name.startswith("/api/iv_msgs/")
        or name.startswith("/awapi/")
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node_name", type=str, default="list_apis")
    parser.add_argument("--namespace", type=str, default="tier4_autoware_api_extension")
    parser.add_argument("--wait_time", type=float, default=1.0)
    args = parser.parse_args()
    with RosNode(args.node_name, args.namespace, args.wait_time) as node:
        for name, types in node.get_topic_names_and_types():
            if is_external_api(name):
                print(name)
        for name, types in node.get_service_names_and_types():
            if is_external_api(name):
                print(name, types)


if __name__ == "__main__":
    main()
