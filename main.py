import argparse
import asyncio
import pathlib
import sys


def _set_project_path() -> None:
    project_dir = pathlib.Path(__file__).resolve().parent
    beachbot_parent_dir = project_dir / "src/python"
    sys.path.append(str(beachbot_parent_dir))


if __name__ == "__main__":
    # Sets the project path so we can find the orchestration module.
    _set_project_path()
    parser = argparse.ArgumentParser(description="Entry point to run nodes.")
    parser.add_argument(
        "profile", type=str, choices=["ui", "rc"], help="Name of profile to run."
    )
    args = parser.parse_args()
    from orchestration import launch

    orchestrator = launch.Orchestrator(args.profile)
    try:
        asyncio.run(orchestrator.run())
    except KeyboardInterrupt:
        # Prevent ^C or ^Z from being printed
        sys.stderr.write("\r")
    finally:
        orchestrator.stop()
