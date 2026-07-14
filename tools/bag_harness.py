#!/usr/bin/env python3
"""bag harness: read a scenario bag, align ground-truth actor poses
to depth frames, and self-check by projecting GT into the image.

Usage:
    python3 bag_harness.py bags/scenario_1_linear --out output/stage0

Importable API for later stage validators:
    from bag_harness import FrameIterator
    for frame in FrameIterator('bags/scenario_1_linear'):
        frame.t          # timestamp (float seconds)
        frame.depth      # np.ndarray float32 (H, W), meters; NaN/inf = no return
        frame.gt_pos     # np.ndarray (3,) actor position in world frame
"""

import argparse
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray

# ---------------------------------------------------------------------------
# Things you MUST check/adjust for your setup
# ---------------------------------------------------------------------------
DEPTH_TOPIC = "/camera/depth_image"
GT_TOPIC = "/world/default/dynamic_pose/info"
INFO_TOPIC = "/camera/camera_info"

# Which entry in the PoseArray is the actor. Run with --inspect-gt first:
# it prints how much each index moved over the bag; the actor is the one
# with large displacement. Then set this.
ACTOR_INDEX = 0

# Robot spawn pose in world frame (must match your launch file's spawn args)
ROBOT_XYZ = np.array([-2.0, -0.5, 0.1])
ROBOT_YAW = 0.5

# Camera link pose relative to base (from turtlebot3_waffle model.sdf,
# camera_rgb_frame <pose>)
CAM_XYZ_IN_BASE = np.array([0.069, -0.047, 0.107])
# ---------------------------------------------------------------------------


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Body frame (x fwd, y left, z up) -> optical frame (z fwd, x right, y down).
# Standard camera convention; if the projected dot lands mirrored or rotated,
# this is the first thing to revisit.
R_BODY_TO_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


def world_to_optical(p_world: np.ndarray) -> np.ndarray:
    """World point -> camera optical frame, using the hardcoded static chain.
    world -> base_link -> camera body -> camera optical."""
    R_wb = rot_z(ROBOT_YAW)                    # base orientation in world
    p_base = R_wb.T @ (p_world - ROBOT_XYZ)    # into base frame
    p_cam_body = p_base - CAM_XYZ_IN_BASE      # camera body frame (same orientation as base)
    return R_BODY_TO_OPTICAL @ p_cam_body


def project(p_world: np.ndarray, K: np.ndarray):
    """World point -> pixel (u, v), or None if behind the camera."""
    p = world_to_optical(p_world)
    if p[2] <= 0.05:
        return None
    u = K[0, 0] * p[0] / p[2] + K[0, 2]
    v = K[1, 1] * p[1] / p[2] + K[1, 2]
    return int(round(u)), int(round(v))


def depth_msg_to_np(msg: Image) -> np.ndarray:
    if msg.encoding != "32FC1":
        raise ValueError(f"Expected 32FC1 depth, got {msg.encoding}")
    return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------

def read_bag(bag_path: str):
    """Single pass over the bag. Returns (depth_frames, gt_samples, K).
    depth_frames: list of (t, np depth image)
    gt_samples:   list of (t, np array (N, 3) of all pose positions)
    K:            3x3 intrinsics from the first CameraInfo
    """
    # storage_id: 'mcap' is the Jazzy default; 'sqlite3' for older bags.
    # ros2 bag info <bag> tells you which. Try mcap first, fall back.
    reader = SequentialReader()
    for storage_id in ("mcap", "sqlite3"):
        try:
            reader.open(StorageOptions(uri=bag_path, storage_id=storage_id),
                        ConverterOptions("", ""))
            break
        except Exception:
            reader = SequentialReader()
    else:
        raise RuntimeError(f"Could not open bag at {bag_path}")

    depth_frames, gt_samples, K = [], [], None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == DEPTH_TOPIC:
            msg = deserialize_message(raw, Image)
            depth_frames.append((stamp_to_sec(msg.header.stamp),
                                 depth_msg_to_np(msg)))
        elif topic == GT_TOPIC:
            msg = deserialize_message(raw, PoseArray)
            positions = np.array([[p.position.x, p.position.y, p.position.z]
                                  for p in msg.poses])
            gt_samples.append((stamp_to_sec(msg.header.stamp), positions))
        elif topic == INFO_TOPIC and K is None:
            msg = deserialize_message(raw, CameraInfo)
            K = np.array(msg.k).reshape(3, 3)

    depth_frames.sort(key=lambda x: x[0])
    gt_samples.sort(key=lambda x: x[0])
    if not depth_frames:
        raise RuntimeError(f"No messages on {DEPTH_TOPIC}")
    if not gt_samples:
        raise RuntimeError(f"No messages on {GT_TOPIC}")
    if K is None:
        raise RuntimeError(f"No CameraInfo on {INFO_TOPIC}")
    return depth_frames, gt_samples, K


def interpolate_gt(gt_samples, t: float, index: int) -> np.ndarray:
    """Actor position at time t, linearly interpolated between GT samples."""
    times = np.array([s[0] for s in gt_samples])
    i = int(np.searchsorted(times, t))
    if i <= 0:
        return gt_samples[0][1][index]
    if i >= len(gt_samples):
        return gt_samples[-1][1][index]
    t0, p0 = gt_samples[i - 1][0], gt_samples[i - 1][1][index]
    t1, p1 = gt_samples[i][0], gt_samples[i][1][index]
    a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return (1 - a) * p0 + a * p1


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    t: float
    depth: np.ndarray
    gt_pos: np.ndarray


class FrameIterator:
    """Yields time-aligned (depth frame, GT actor position) pairs."""

    def __init__(self, bag_path: str, actor_index: int = ACTOR_INDEX):
        self.depth_frames, self.gt_samples, self.K = read_bag(bag_path)
        self.actor_index = actor_index

    def __iter__(self):
        for t, depth in self.depth_frames:
            yield Frame(t, depth, interpolate_gt(self.gt_samples, t,
                                                 self.actor_index))

    def __len__(self):
        return len(self.depth_frames)


# ---------------------------------------------------------------------------
# Self-checks
# ---------------------------------------------------------------------------

def inspect_gt(bag_path: str):
    """Print per-index displacement so you can identify the actor."""
    _, gt_samples, _ = read_bag(bag_path)
    first, last = gt_samples[0][1], gt_samples[-1][1]
    n = min(len(first), len(last))
    print(f"{len(gt_samples)} GT samples, {n} poses per sample")
    for i in range(n):
        span = np.ptp([s[1][i] for s in gt_samples if len(s[1]) > i], axis=0)
        print(f"  index {i}: start {first[i].round(2)}, "
              f"total motion range {span.round(2)}  "
              f"{'<-- probably the actor' if np.linalg.norm(span[:2]) > 1.0 else ''}")


def dot_on_blob(bag_path: str, out_dir: str, every: int = 5):
    """The Stage 0 pass check: draw projected GT on depth frames."""
    it = FrameIterator(bag_path)
    os.makedirs(out_dir, exist_ok=True)
    in_view = 0
    total = 0
    for n, frame in enumerate(it):
        px = project(frame.gt_pos, it.K)
        total += 1
        h, w = frame.depth.shape
        visible = px is not None and 0 <= px[0] < w and 0 <= px[1] < h
        if visible:
            in_view += 1
        if n % every:
            continue
        # Render depth to an 8-bit image for viewing
        d = frame.depth.copy()
        d[~np.isfinite(d)] = 8.0
        img = cv2.cvtColor((np.clip(d / 8.0, 0, 1) * 255).astype(np.uint8),
                           cv2.COLOR_GRAY2BGR)
        if visible:
            cv2.circle(img, px, 6, (0, 0, 255), 2)
        cv2.putText(img, f"t={frame.t:.2f} gt={frame.gt_pos.round(2)}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imwrite(os.path.join(out_dir, f"frame_{n:04d}.png"), img)
    print(f"{total} frames; GT projected inside image bounds on {in_view} "
          f"({100.0 * in_view / max(total, 1):.0f}%). Annotated frames in {out_dir}/")
    print("PASS if the red dot rides the actor blob whenever the actor is in view.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", help="path to the bag directory")
    ap.add_argument("--out", default="output/stage0")
    ap.add_argument("--inspect-gt", action="store_true",
                    help="print GT pose indices and motion, to find the actor")
    args = ap.parse_args()

    if args.inspect_gt:
        inspect_gt(args.bag)
        sys.exit(0)
    dot_on_blob(args.bag, args.out)