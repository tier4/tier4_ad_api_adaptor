#!/usr/bin/bash
cd resource/proto
find . -name *.proto | xargs protoc --python_out=../python --js_out=../js
find . -name *.proto | xargs npx pbjs -t static-module -w commonjs -o ../autoware_protobuf.js --keep-case
