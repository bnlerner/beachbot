import pathlib

UBLOX_SERIAL = "/dev/ttyACM0"


def get_root_project_directory() -> pathlib.Path:
    """Gets the root directory (beachbot) of this project's repository."""
    current_module = pathlib.Path(__file__).resolve()
    directories = current_module.parts
    idx = directories.index("src")

    return pathlib.Path().joinpath(*directories[:idx])
