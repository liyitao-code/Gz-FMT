#!/usr/bin/env bash

BUILD_DIR="/home/liyitao/workspace/gz_lastest/build/"
rm gcov/*.json.gz

find $BUILD_DIR -name '*.gcda' -delete

