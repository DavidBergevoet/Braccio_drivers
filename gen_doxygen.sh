#!/usr/bin/env bash

echo "      Removing the current doxygen files      "

rm -rf doxygen > /dev/null 2>&1

echo "      Generating doxygen                      "

doxygen doxygen_config.txt > /dev/null 2>&1