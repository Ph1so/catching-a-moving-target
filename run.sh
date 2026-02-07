#!/bin/bash

TYPE=$1
MAP=$2

if [ -z "$TYPE" ] || [ -z "$MAP" ]; then
  echo "Usage: ./run.sh <u|g> <map_number>"
  exit 1
fi

case "$TYPE" in
  u)
    DIR="undergrad"
    ;;
  g)
    DIR="grad"
    ;;
  *)
    echo "First argument must be 'u' (undergrad) or 'g' (grad)"
    exit 1
    ;;
esac

g++ runtest.cpp planner.cpp -o planner || exit 1
./planner "$DIR/map${MAP}.txt" || exit 1
python visualizer.py "$DIR/map${MAP}.txt"
