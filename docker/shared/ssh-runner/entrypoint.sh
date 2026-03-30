#!/usr/bin/env sh
set -eu

if [ "$#" -eq 0 ]; then
  echo "usage: ssh-runner-entrypoint <command> [args...]" >&2
  exit 1
fi

exec "$@"
