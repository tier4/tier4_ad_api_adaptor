#!/usr/bin/bash
cd resource/proto

# python
find . -name *.proto | xargs protoc --python_out=../python

# nodejs
find . -name *.proto | xargs npx pbjs -t static-module -w commonjs -o ../autoware_protobuf.js --keep-case
npx pbts -o ../autoware_protobuf.d.ts ../autoware_protobuf.js

# browser
find . -name *.proto | xargs npx pbjs -t json -o ../autoware_protobuf.json --keep-case
find . -name *.proto | xargs npx pbjs -t proto3 -o ../autoware_protobuf.proto --keep-case
