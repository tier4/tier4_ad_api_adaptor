#!/usr/bin/bash
cd resource/proto
find . -name *.proto | xargs protoc --python_out=../python