"""Shared UDP realtime feedback helpers for RM75B teleop modes."""

import time

import numpy as np


def enable_udp_feedback(arm, cfg):
    arm.enable_udp_realtime_feedback(
        cfg.UDP_TARGET_IP,
        cfg.UDP_TARGET_PORT,
        cfg.UDP_CYCLE_MS,
        cfg.UDP_FORCE_COORDINATE,
    )
    arm.register_udp_feedback_callback()


def read_udp_feedback(arm, timeout_s):
    state, age = arm.get_latest_udp_state(timeout_s)

    # state check -----------------
    if state is None:
        print("[WARN] UDP feedback unavailable or timed out")
        return None, None, age

    force_sensor = state.get("force_sensor")
    if force_sensor is None:
        print(f"[WARN] UDP feedback missing force_sensor. keys={list(state.keys())}")
        return None, None, age

    waypoint = state.get("waypoint")
    if waypoint is None:
        print(f"[WARN] UDP feedback missing waypoint. keys={list(state.keys())}")
        return None, None, age
    # state check -----------------

    # force check -----------------
    values = force_sensor.get("zero_force")
    if values is None:
        raise RuntimeError(f"UDP force_sensor missing zero_force field. keys={list(force_sensor.keys())}")

    force6 = np.array(values[:6], dtype=float)
    if force6.shape[0] != 6:
        raise RuntimeError(f"Invalid UDP zero_force length: {force6.shape[0]}, values={values}")
    # force check -----------------

    # pose check -----------------
    position = waypoint.get("position")
    euler = waypoint.get("euler")
    if position is None or euler is None:
        raise RuntimeError(f"UDP waypoint missing position/euler field. keys={list(waypoint.keys())}")

    pose = np.array([
        position["x"],
        position["y"],
        position["z"],
        euler["rx"],
        euler["ry"],
        euler["rz"],
    ], dtype=float)

    if pose.shape[0] != 6:
        raise RuntimeError(f"Invalid UDP pose length: {pose.shape[0]}, values={pose}")
    # pose check -----------------

    return force6, pose, age


def wait_for_udp_pose(arm, timeout_s, wait_s=1.0, poll_s=0.005):
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        force6, current_pose, feedback_age = read_udp_feedback(arm, timeout_s)
        if current_pose is not None:
            return current_pose
        time.sleep(poll_s)
    return None
