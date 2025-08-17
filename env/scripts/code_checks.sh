#!/bin/bash
DIRNAME="${PWD##*/}"
if [ "$DIRNAME" != "beachbot" ]; then
    echo "Navigate to the beachbot directory prior to running this command.";
    exit 1;
fi
SEARCH_PATH=$1
if [ -z "$SEARCH_PATH" ]; then
    echo "Usage: $0 <search_path>";
    exit 1;
fi

mypy --config-file "$PWD/env/conf_files/mypy.ini" "$SEARCH_PATH"
ruff check "$SEARCH_PATH" --config "$PWD/env/conf_files/ruff.toml" --no-cache --fix
ruff format "$SEARCH_PATH" --config "$PWD/env/conf_files/ruff.toml" --no-cache
