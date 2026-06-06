#!/usr/bin/bash

ref_dir="../proto/referee"
sim_dir="../proto/simulation"
output_dir="../vss/vision/proto_generated/"

if [ ! -d "$output_dir" ]; then
    mkdir -p "$output_dir"
fi

for file in "$ref_dir"/*.proto; do
    protoc --proto_path="$ref_dir" --python_out="$output_dir" "$file"
done

for file in "$sim_dir"/*.proto; do
    protoc --proto_path="$sim_dir" --python_out="$output_dir" "$file"
done

