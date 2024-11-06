import argparse
import asyncio
import pathlib
import sys

# TODO: May need to run these at startup to confirm motor configs
# python3 can_restore_config.py --channel can0 --node-id 0 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/front_left.json --save-config
# python3 can_restore_config.py --channel can0 --node-id 1 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/front_right.json --save-config
# python3 can_restore_config.py --channel can0 --node-id 2 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/rear_left.json --save-config
# python3 can_restore_config.py --channel can0 --node-id 3 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/rear_right.json --save-config


def _set_project_path() -> None:
    project_dir = pathlib.Path(__file__).resolve().parent
    beachbot_parent_dir = project_dir / "src/python"
    sys.path.append(str(beachbot_parent_dir))


if __name__ == "__main__":
    _set_project_path()
    parser = argparse.ArgumentParser(description="Entry point to run nodes.")
    parser.add_argument(
        "--profile",
        type=str,
        required=True,
        choices=["hw", "rc"],
        help="Name of profile to run.",
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
