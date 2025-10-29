#!/bin/sh

can_port=""
dir=""
ids=""
uuids=""
if [ -z "$1" -o "$1" = "bench" ]; then
  echo "Config: Bench"
  can_port="can0"
  dir="bench"
  ids=`seq 11 16`
elif [ "$1" = "explorer" ]; then
  echo "Config: Explorer"
  can_port="can1"
  dir="explorer"
  ids=`seq 11 16`
else
  echo "Usage: $0 [explorer|bench]. Abort"
  exit 0
fi
shift
