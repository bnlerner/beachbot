import time
from typing import Generator

import pytest
from config import robot_config
from drivers.can import messages as can_messages
from typing_helpers import req

from ipc import messages, pubsub, registry

_TARGET_HZ = 5_000
_TARGET_LATENCY = 5e-4  # in seconds


@pytest.fixture
def publisher() -> Generator[pubsub.Publisher, None, None]:
    pub = pubsub.Publisher[messages.MotorCommandMessage](
        registry.NodeIDs.TEST0, registry.Channels.TEST
    )
    try:
        yield pub
    finally:
        pub.close()


@pytest.fixture
def subscriber() -> Generator[pubsub.Subscriber, None, None]:
    sub = pubsub.Subscriber(
        registry.NodeIDs.TEST1, registry.Channels.TEST, _process_msg
    )
    try:
        yield sub
    finally:
        sub.close()


def _process_msg(msg: messages.MotorCommandMessage) -> None:
    latency = time.perf_counter() - req(msg.creation)
    _ = can_messages.SetVelocityMessage(
        msg.motor.node_id, velocity=msg.velocity, torque=msg.feedforward_torque
    )
    assert latency <= _TARGET_LATENCY


def _gen_sample_msg() -> messages.MotorCommandMessage:
    return messages.MotorCommandMessage(
        motor=robot_config.Motor(
            node_id=10,
            location=robot_config.DrivetrainLocation.FRONT_LEFT,
            torque_constant=10,
            continous_current=10,
        ),
        velocity=1.0,
    )


async def test_pubsub_speed(
    subscriber: pubsub.Subscriber, publisher: pubsub.Publisher
) -> None:
    start_time = time.perf_counter()
    sample_msg = _gen_sample_msg()
    for _ in range(_TARGET_HZ):
        publisher.publish(sample_msg)
        msg = await subscriber.wait_for_message()
        assert msg is not None
        _process_msg(msg)

    run_time = time.perf_counter() - start_time
    assert run_time < 1.0
