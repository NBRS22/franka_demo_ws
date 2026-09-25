# Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Lightweight ZMQ client for GraspGen server.

Only depends on pyzmq, msgpack, msgpack-numpy, and numpy — no torch / CUDA needed.
This makes it suitable for running on robot controllers or edge devices.

Usage:
    from grasp_gen.serving.zmq_client import GraspGenClient

    client = GraspGenClient("localhost", 5558)
    grasps, confidences = client.infer(point_cloud)
"""

import logging
import time
from typing import Optional

import numpy as np
import zmq
import msgpack
import msgpack_numpy

msgpack_numpy.patch()

logger = logging.getLogger(__name__)


class GraspGenClient:
    """Client that connects to a GraspGen ZMQ server for remote grasp inference."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 5558,
        timeout_ms: int = 60_000,
        wait_for_server: bool = True,
        retry_interval_s: float = 2.0,
    ) -> None:
        self._addr = f"tcp://{host}:{port}"
        self._timeout_ms = timeout_ms
        self._ctx = zmq.Context()
        self._socket: Optional[zmq.Socket] = None
        self._server_metadata: Optional[dict] = None

        if wait_for_server:
            self._wait_for_server(retry_interval_s)

    def _create_socket(self) -> zmq.Socket:
        sock = self._ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self._addr)
        return sock

    def _wait_for_server(self, retry_interval_s: float) -> None:
        logger.info("Waiting for GraspGen server at %s ...", self._addr)
        while True:
            try:
                self._socket = self._create_socket()
                self._server_metadata = self._request({"action": "metadata"})
                logger.info(
                    "Connected to GraspGen server: %s", self._server_metadata
                )
                return
            except (zmq.error.Again, zmq.error.ZMQError):
                logger.info("Server not ready, retrying in %.1fs ...", retry_interval_s)
                if self._socket is not None:
                    self._socket.close()
                    self._socket = None
                time.sleep(retry_interval_s)

    def _ensure_connected(self) -> None:
        if self._socket is None:
            self._socket = self._create_socket()

    def _request(self, payload: dict) -> dict:
        self._ensure_connected()
        self._socket.send(msgpack.packb(payload, use_bin_type=True))
        raw = self._socket.recv()
        response = msgpack.unpackb(raw, raw=False)
        if "error" in response:
            raise RuntimeError(f"Server error: {response['error']}")
        return response

    @property
    def server_metadata(self) -> Optional[dict]:
        return self._server_metadata

    def health_check(self) -> bool:
        try:
            resp = self._request({"action": "health"})
            return resp.get("status") == "ok"
        except Exception:
            return False

    def get_metadata(self) -> dict:
        return self._request({"action": "metadata"})

    def infer(
        self,
        point_cloud: np.ndarray,
        *,
        planner: str = "diffusion",
        grasp_threshold: float = -1.0,
        num_grasps: int = 200,
        topk_num_grasps: int = -1,
        min_grasps: int = 40,
        max_tries: int = 6,
        remove_outliers: bool = True,
        scene_point_cloud: Optional[np.ndarray] = None,
        collision_threshold: float = 0.02,
        max_scene_points: int = 8192,
        moe_num_yaws: int = 36,
        moe_z_offsets_cm: tuple = (-8, -6, -4, -2, 0),
        moe_outlier_threshold: float = 0.014,
        moe_outlier_k: int = 20,
        moe_obb_mode: str = "advanced",
        moe_skip_obb_rule: str = "auto",
        moe_obb_density: str = "dense-topandside",
        moe_obb_position_spacing_cm: float = 1.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Send a point cloud to the server and receive grasp predictions.

        Args:
            point_cloud: (N, 3) float32 array of object points.
            planner: "diffusion" (raw sampler, default) or "graspmoe" (diffusion
                grasps unioned with deterministic OBB-swept candidates -- top
                face + sides by default -- both scored by the discriminator;
                see grasp_gen/samplers/graspmoe.py). Unlike the diffusion
                sampler, graspmoe is aware of "up" (it sweeps the object's own
                oriented bounding box), so it reliably includes top-down
                candidates even when the diffusion model alone favors lateral
                grasps for a given object shape.
            grasp_threshold: Min confidence to keep. -1.0 returns top-k instead.
            num_grasps: Number of grasps the diffusion model should sample.
            topk_num_grasps: Return only top-k grasps (-1 = use threshold).
                For planner="graspmoe" this caps the union of both branches.
            min_grasps: Minimum grasps before retrying (planner="diffusion" only).
            max_tries: Max inference retries on the server (planner="diffusion" only).
            remove_outliers: Whether to filter point cloud outliers (planner="diffusion"
                only -- graspmoe always runs its own outlier removal, see
                moe_outlier_threshold/moe_outlier_k).
            scene_point_cloud: Optional (S, 3) float32 array of the surrounding
                scene (table, other objects, ...) with the target object's own
                points already excluded. When provided, the server drops any
                returned grasp whose gripper mesh would collide with it.
            collision_threshold: Distance (meters) under which a gripper surface
                sample counts as colliding with scene_point_cloud.
            max_scene_points: Server-side random-downsample cap for scene_point_cloud
                before collision checking.
            moe_num_yaws: planner="graspmoe" only -- yaw samples per OBB face.
            moe_z_offsets_cm: planner="graspmoe" only -- standoff offsets (cm)
                swept along each face's approach direction.
            moe_outlier_threshold / moe_outlier_k: planner="graspmoe" only --
                outlier-removal hyperparameters applied once before both branches.
            moe_obb_mode: planner="graspmoe" only -- "advanced" (SOR + hull +
                rotating calipers) or "pca".
            moe_skip_obb_rule: planner="graspmoe" only -- "auto" skips the OBB
                branch when every extent exceeds the gripper's jaw width (object
                too big to enclose); "never" always runs it.
            moe_obb_density: planner="graspmoe" only -- "sparse" (single
                position per face), "dense" (positions swept along the long
                axis), or "dense-topandside" (dense positions on the top face
                AND all 4 side faces -- the richest option, includes top-down).
            moe_obb_position_spacing_cm: planner="graspmoe" only -- spacing (cm)
                for position sweeps in "dense"/"dense-topandside" modes.

        Returns:
            grasps: (M, 4, 4) float32 array of 6-DOF grasp poses.
            confidences: (M,) float32 array of grasp confidence scores.
        """
        point_cloud = np.asarray(point_cloud, dtype=np.float32)
        if point_cloud.ndim != 2 or point_cloud.shape[1] != 3:
            raise ValueError(f"point_cloud must be (N, 3), got {point_cloud.shape}")

        payload = {
            "action": "infer",
            "point_cloud": point_cloud,
            "planner": planner,
            "grasp_threshold": grasp_threshold,
            "num_grasps": num_grasps,
            "topk_num_grasps": topk_num_grasps,
            "min_grasps": min_grasps,
            "max_tries": max_tries,
            "remove_outliers": remove_outliers,
        }
        if planner == "graspmoe":
            payload["moe_num_yaws"] = moe_num_yaws
            payload["moe_z_offsets_cm"] = moe_z_offsets_cm
            payload["moe_outlier_threshold"] = moe_outlier_threshold
            payload["moe_outlier_k"] = moe_outlier_k
            payload["moe_obb_mode"] = moe_obb_mode
            payload["moe_skip_obb_rule"] = moe_skip_obb_rule
            payload["moe_obb_density"] = moe_obb_density
            payload["moe_obb_position_spacing_cm"] = moe_obb_position_spacing_cm
        if scene_point_cloud is not None:
            scene_point_cloud = np.asarray(scene_point_cloud, dtype=np.float32)
            if scene_point_cloud.ndim != 2 or scene_point_cloud.shape[1] != 3:
                raise ValueError(
                    f"scene_point_cloud must be (S, 3), got {scene_point_cloud.shape}"
                )
            payload["scene_point_cloud"] = scene_point_cloud
            payload["collision_threshold"] = collision_threshold
            payload["max_scene_points"] = max_scene_points

        response = self._request(payload)
        grasps = np.asarray(response["grasps"], dtype=np.float32)
        confidences = np.asarray(response["confidences"], dtype=np.float32)
        return grasps, confidences

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._ctx.term()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()
