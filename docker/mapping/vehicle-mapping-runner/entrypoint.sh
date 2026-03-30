#!/usr/bin/env sh
set -eu

if [ "$#" -eq 0 ]; then
  echo "usage: vehicle-mapping-runner-entrypoint <command> [args...]" >&2
  exit 1
fi

if [ -f /opt/ros/foxy/setup.sh ]; then
  set +u
  . /opt/ros/foxy/setup.sh
  set -u
fi

exec "$@"
