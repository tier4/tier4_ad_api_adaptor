#!/usr/bin/env python3
import argparse
import pathlib

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

    def get_external_api_names(self):
        apis = []
        for name, types in self.get_topic_names_and_types():
            if self.is_external_api(name):
                apis.append(name)
        for name, types in self.get_service_names_and_types():
            if self.is_external_api(name):
                apis.append(name)
        return apis

    def get_topic_names_and_types(self):
        return self.__remove_builtins(self.node.get_topic_names_and_types())

    def get_service_names_and_types(self):
        return self.__remove_builtins(self.node.get_service_names_and_types())

    @classmethod
    def __remove_builtins(cls, names_and_types):
        for name, types in names_and_types:
            if len(types) == 1:
                if not cls.__is_builtin_type(types[0]):
                    yield name, types

    @classmethod
    def __is_builtin_type(cls, type_name):
        builtins = ["rcl_interfaces", "composition_interfaces"]
        for prefix in builtins:
            if type_name.startswith(prefix):
                return True
        return False

    @staticmethod
    def is_external_api(name):
        return any(
            (
                name.startswith("/api/external/"),
                name.startswith("/api/iv_msgs/"),
                name.startswith("/awapi/"),
            )
        )


class API:
    def __init__(self, name, *, doc=False, ros=False, eol=None, release=None):
        self.name = name
        self.doc = doc
        self.ros = ros
        self.eol = eol
        self.release = release


def get_readme_path():
    return pathlib.Path(__file__).parents[1] / "README.md"


def strip_markdown_link(text):
    if text.startswith("[") and text.endswith(")"):
        return text.split("](")[0][1:]
    return text


def is_header(data):
    if data == "Name":
        return True
    if set(data) == set("-"):
        return True
    return False


def find_dummy(line):
    return None


def find_api_extension(line):
    parts = line.split("|")
    if len(parts) == 6:
        name = strip_markdown_link(parts[3].strip())
        info = strip_markdown_link(parts[1].strip())
        if not is_header(name):
            return API(name, doc=True, release=info)


def find_deprecated_api(line):
    parts = line.split("|")
    if len(parts) == 6:
        name = strip_markdown_link(parts[3].strip())
        info = strip_markdown_link(parts[1].strip())
        if not is_header(name):
            return API(name, doc=True, eol=info)


def find_deprecated_awapi(line):
    parts = line.split("|")
    if len(parts) == 4:
        name = strip_markdown_link(parts[2].strip())
        info = strip_markdown_link(parts[1].strip())
        if not is_header(name):
            return API(name, doc=True, eol=info)


def list_docs_apis(args):
    find_func_dict = {
        "## TIER IV Autoware API": find_api_extension,
        "## Deprecated API": find_deprecated_api,
        "## Deprecated API (AWAPI)": find_deprecated_awapi,
    }
    find_func = find_dummy
    apis = []
    with args.path.open() as fp:
        for line in fp:
            line = line.strip()
            if line.startswith("## "):
                find_func = find_func_dict.get(line, find_dummy)
            else:
                apis.append(find_func(line))
    return [api for api in apis if api is not None]


def list_ros_apis(args):
    apis = []
    with RosNode(args.node_name, args.namespace, args.wait_time) as node:
        for name in node.get_external_api_names():
            apis.append(API(name, ros=True))
    return apis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node_name", type=str, default="list_apis")
    parser.add_argument("--namespace", type=str, default="tier4_autoware_api_extension")
    parser.add_argument("--wait_time", type=float, default=1.0)
    parser.add_argument("--path", type=pathlib.Path, default=get_readme_path())
    parser.add_argument("--exclude-eol", nargs="+", default=[])
    args = parser.parse_args()
    apis = {}
    for api in list_docs_apis(args):
        apis[api.name] = api
    for api in list_ros_apis(args):
        if api.name in apis:
            apis[api.name].ros = True
        else:
            apis[api.name] = api

    for api in sorted(apis.values(), key=lambda api: api.name):
        if api.eol not in args.exclude_eol:
            doc = "doc" if api.doc else "   "
            ros = "ros" if api.ros else "   "
            print(f"[{doc} {ros}] {str(api.eol):<8} {api.name}")


if __name__ == "__main__":
    main()
