#!/bin/bash
pytest --capture=tee-sys --asyncio-mode=auto --color=yes --last-failed \
        # uvloop can raise a warning when the connection is refused in UDP stream
        -W error::pytest.PytestUnraisableExceptionWarning \
        # Casadi is importing ABCs from collections instead of collections.abc
        -W ignore::DeprecationWarning:casadi.tools.structure3 \
        --ignore-glob='*system_test.py' --ignore-glob='*hw_test.py' \
        -vv
