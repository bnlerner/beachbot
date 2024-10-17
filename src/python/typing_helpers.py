import traceback
from typing import NoReturn, Optional, TypeVar, overload

T = TypeVar("T")


@overload
def req(optional: None) -> NoReturn:
    ...


@overload
def req(optional: T) -> T:
    ...


def req(optional: Optional[T]) -> T:
    if optional is None:
        offending_frame: traceback.FrameSummary = traceback.extract_stack()[-2]

        filename = offending_frame.filename
        line_number = offending_frame.lineno
        line = offending_frame.line

        raise RuntimeError(
            f"Failed the required optional check: {line} at {filename} line: {line_number}."
        )

    return optional
