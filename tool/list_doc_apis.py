#!/usr/bin/env python3
import argparse
import pathlib


def get_readme_path():
    return pathlib.Path(__file__).parents[1] / "README.md"


def strip_markdown_link(text):
    if text.startswith("[") and text.endswith(")"):
        return text.split("](")[0][1:]
    return text


def find_dummy(line):
    return None


def find_api_extension(line):
    parts = line.split("|")
    if len(parts) == 6:
        print(parts[1].strip(), strip_markdown_link(parts[3].strip()))


def find_deprecated_api(line):
    parts = line.split("|")
    if len(parts) == 6:
        print(parts[1].strip(), strip_markdown_link(parts[3].strip()))


def find_deprecated_awapi(line):
    parts = line.split("|")
    if len(parts) == 4:
        print(parts[1].strip(), strip_markdown_link(parts[2].strip()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=pathlib.Path, default=get_readme_path())
    args = parser.parse_args()
    apis = []
    find_func_dict = {
        "## TIER IV Autoware API": find_api_extension,
        "## External API": find_deprecated_api,
        "## Deprecated API (AWAPI)": find_deprecated_awapi,
    }
    find_func = find_dummy
    with args.path.open() as fp:
        for line in fp:
            line = line.strip()
            if line.startswith("## "):
                find_func = find_func_dict.get(line, find_dummy)
            else:
                apis.append(find_func(line))

    apis = [api for api in apis if api is not None]
    for api in apis:
        print(api)


if __name__ == "__main__":
    main()
