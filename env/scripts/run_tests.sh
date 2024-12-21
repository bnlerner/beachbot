#!/bin/bash
if [[ "$1" == "--unit" ]]; then
    pytest --capture=tee-sys --asyncio-mode=auto --color=yes -vv --ignore-glob='*system_test.py' $2
elif [[ "$1" == "--system" ]]; then
    pytest --capture=tee-sys --asyncio-mode=auto --color=yes -vv --ignore-glob='*unit_test.py' $2
else
    pytest --capture=tee-sys --asyncio-mode=auto --color=yes -vv $1
fi
