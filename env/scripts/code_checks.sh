#!/bin/bash
DIRNAME="${PWD##*/}"
if [ "$DIRNAME" != "beachbot" ]; then
    echo "Navigate to the beachbot directory prior to running this command.";
    exit 1;
fi
mypy --config-file "$PWD/env/conf_files/mypy.ini" .
ruff check . --config "$PWD/env/conf_files/ruff.toml" --no-cache --fix
ruff format . --config "$PWD/env/conf_files/ruff.toml" --no-cache
