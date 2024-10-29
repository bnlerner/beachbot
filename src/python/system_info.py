import pathlib
from datetime import datetime, timezone


def get_root_project_directory() -> pathlib.Path:
    """Gets the root directory (beachbot) of this project's repository."""
    current_module = pathlib.Path(__file__).resolve()
    directories = current_module.parts
    idx = directories.index("src")

    return pathlib.Path().joinpath(*directories[:idx])


def timestamp() -> str:
    """Current timestamp as a string."""
    return datetime.now(tz=timezone.utc).isoformat()
