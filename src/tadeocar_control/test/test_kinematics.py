"""What the 4WS kinematics node owes a wheel that is stopped or pointed sideways.

Two defects, both found by watching the robot rather than by reading the code,
and both invisible to Nav2's runs, which drive forwards and turn:

  * A sideways command folded into a fixed [-90, 90] degrees flips between +90
    and -90 on the slightest change in the command, so a heading correction
    that changed sign swung the four servos half a turn back and forth.
  * A stopped wheel held its MEASURED angle, which gives the servo no error to
    act on, so anything that turned the wheel turned it for good.

Run directly with ``python3 -m pytest`` or through ``colcon test``.
"""

import math

import pytest
import rclpy

from tadeocar_control.fourws_kinematics_node import FourWSKinematicsNode, WHEELS

LIMIT = 2.356


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = FourWSKinematicsNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def at_rest(node):
    node.held_steering = {w: 0.0 for w in WHEELS}
    node.current_steering = {w: 0.0 for w in WHEELS}
    node.heading = None
    node.crab_reference = None
    node.mode = 'omnidirectional'


def _send(node, steering):
    """What _publish remembers, without needing anyone listening."""
    node.held_steering = dict(steering)


@pytest.mark.parametrize('deg', range(-180, 181, 5))
def test_from_rest_the_fold_is_the_quarter_turn_one(node, deg):
    angle, direction = node._fold_steering(math.radians(deg), 0.0)
    assert -math.pi / 2 - 1e-9 <= angle <= math.pi / 2 + 1e-9
    # same velocity: pointing angle with direction s is pointing deg
    expected = math.radians(deg) if direction > 0 else math.radians(deg) + math.pi
    assert math.isclose(math.cos(angle), math.cos(expected), abs_tol=1e-9)
    assert math.isclose(math.sin(angle), math.sin(expected), abs_tol=1e-9)


@pytest.mark.parametrize('reference_deg', [-130, -90, -45, 0, 45, 90, 130])
@pytest.mark.parametrize('deg', range(-180, 181, 15))
def test_the_fold_stays_inside_the_servo_travel(node, reference_deg, deg):
    angle, _ = node._fold_steering(math.radians(deg), math.radians(reference_deg))
    assert abs(angle) <= LIMIT


def test_a_sideways_command_does_not_chatter(node):
    """+90 degrees with a heading correction that keeps changing sign."""
    first = None
    for k in range(40):
        wz = 0.01 if k % 2 else -0.01
        steering, speeds = node._omnidirectional(0.0, 0.3, wz)
        _send(node, steering)
        if first is None:
            first = dict(steering)
            continue
        for w in WHEELS:
            assert abs(steering[w] - first[w]) < math.radians(5), (w, k)
            assert speeds[w] != 0.0


def test_a_sideways_move_still_moves_sideways(node):
    steering, speeds = node._omnidirectional(0.0, 0.3, 0.0)
    for w in WHEELS:
        vx = math.cos(steering[w]) * speeds[w] * node.wheel_radius
        vy = math.sin(steering[w]) * speeds[w] * node.wheel_radius
        assert math.isclose(vx, 0.0, abs_tol=1e-9)
        assert math.isclose(vy, 0.3, abs_tol=1e-9)


def test_a_stopped_wheel_holds_what_it_was_sent_not_what_it_measures(node):
    steering, _ = node._omnidirectional(0.0, 0.3, 0.0)
    _send(node, steering)
    # the tyre unwinds after the stop and turns the wheels
    node.current_steering = {w: a * 0.5 for w, a in steering.items()}
    held, speeds = node._omnidirectional(0.0, 0.0, 0.0)
    assert held == steering
    assert all(s == 0.0 for s in speeds.values())


def test_crab_stopped_holds_what_it_was_sent(node):
    node.mode = 'crab'
    steering, _ = node._crab(0.0, 0.3)
    _send(node, steering)
    node.current_steering = {w: 0.0 for w in WHEELS}
    held, speeds = node._crab(0.0, 0.0)
    assert held == steering
    assert all(s == 0.0 for s in speeds.values())


def test_a_turn_past_the_servo_travel_is_reached_backwards(node):
    """170 degrees cannot be steered to; -10 degrees spun backwards can."""
    angle, direction = node._fold_steering(math.radians(170), 0.0)
    assert math.isclose(angle, math.radians(-10), abs_tol=1e-9)
    assert direction == -1.0
