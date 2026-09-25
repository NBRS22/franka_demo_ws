# Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import time
import logging
from typing import Optional

import numpy as np
import trimesh
import zmq
import msgpack
import msgpack_numpy

msgpack_numpy.patch()

from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg
from grasp_gen.robot import get_gripper_info
from grasp_gen.samplers import run_graspmoe
from grasp_gen.utils.point_cloud_utils import filter_colliding_grasps_fast

logger = logging.getLogger(__name__)

_NUM_GRIPPER_COLLISION_SAMPLES = 2000


class GraspGenZMQServer:
    """ZMQ server that wraps GraspGenSampler for remote grasp inference.

    Protocol (msgpack over ZMQ REP socket):
        Request:  {"action": "infer", "point_cloud": ndarray(N,3), ...params}
                  {"action": "metadata"}
                  {"action": "health"}
        Response: msgpack-encoded dict with results or error.
    """

    def __init__(
        self,
        gripper_config: str,
        host: str = "0.0.0.0",
        port: int = 5558,
    ) -> None:
        self._host = host
        self._port = port
        self._gripper_config = gripper_config

        logger.info("Loading gripper config from %s", gripper_config)
        self._cfg = load_grasp_cfg(gripper_config)
        self._gripper_name = self._cfg.data.gripper_name
        self._model_name = self._cfg.eval.model_name

        logger.info(
            "Initializing GraspGenSampler (model=%s, gripper=%s)",
            self._model_name,
            self._gripper_name,
        )
        self._sampler = GraspGenSampler(self._cfg)
        logger.info("Model loaded and ready for inference")

        # Pre-sample the gripper's own collision mesh once so per-request collision
        # filtering (scene_point_cloud, cf. _handle_infer) doesn't re-sample it every
        # call. None (collision filtering disabled) if the mesh can't be loaded for
        # this gripper — infer() still works, it just never filters on scene_point_cloud.
        self._gripper_surface_points = None
        try:
            gripper_info = get_gripper_info(self._gripper_name)
            sampled, _ = trimesh.sample.sample_surface(
                gripper_info.collision_mesh, _NUM_GRIPPER_COLLISION_SAMPLES
            )
            self._gripper_surface_points = np.asarray(sampled, dtype=np.float32)
            logger.info(
                "Pre-sampled %d gripper collision surface points for scene_point_cloud filtering",
                len(self._gripper_surface_points),
            )
        except Exception:
            logger.warning(
                "Could not load collision mesh for gripper '%s' — scene_point_cloud "
                "collision filtering will be skipped on every request",
                self._gripper_name,
                exc_info=True,
            )

        self._metadata = {
            "gripper_name": self._gripper_name,
            "model_name": self._model_name,
            "gripper_config": gripper_config,
        }

    def serve_forever(self) -> None:
        ctx = zmq.Context()
        socket = ctx.socket(zmq.REP)
        bind_addr = f"tcp://{self._host}:{self._port}"
        socket.bind(bind_addr)
        logger.info("GraspGen ZMQ server listening on %s", bind_addr)

        try:
            while True:
                raw = socket.recv()
                try:
                    request = msgpack.unpackb(raw, raw=False)
                    response = self._handle(request)
                except Exception as exc:
                    logger.exception("Error handling request")
                    response = {"error": str(exc)}
                socket.send(msgpack.packb(response, use_bin_type=True))
        except KeyboardInterrupt:
            logger.info("Shutting down server")
        finally:
            socket.close()
            ctx.term()

    def _handle(self, request: dict) -> dict:
        action = request.get("action")
        if action == "health":
            return {"status": "ok"}
        if action == "metadata":
            return self._metadata
        if action == "infer":
            return self._handle_infer(request)
        return {"error": f"Unknown action: {action}"}

    def _handle_infer(self, request: dict) -> dict:
        point_cloud = request.get("point_cloud")
        if point_cloud is None:
            return {"error": "Missing required field 'point_cloud'"}

        point_cloud = np.asarray(point_cloud, dtype=np.float32)
        if point_cloud.ndim != 2 or point_cloud.shape[1] != 3:
            return {
                "error": f"point_cloud must be (N, 3), got {point_cloud.shape}"
            }

        planner = str(request.get("planner", "diffusion"))
        if planner not in ("diffusion", "graspmoe"):
            return {
                "error": f"Unknown planner '{planner}' (expected 'diffusion' or 'graspmoe')"
            }

        branch_tags: Optional[list] = None
        skipped_obb: Optional[bool] = None

        t0 = time.monotonic()
        if planner == "graspmoe":
            # OBB-swept candidates (top face + 4 sides by default) unioned with
            # the diffusion sampler, both scored by the same discriminator --
            # see grasp_gen/samplers/graspmoe.py. Defaults mirror
            # scripts/demo_scene_pc.py's --planner graspmoe CLI defaults.
            moe_params = {
                "grasp_threshold": float(request.get("grasp_threshold", -1.0)),
                "num_grasps": int(request.get("num_grasps", 200)),
                "topk_num_grasps": int(request.get("topk_num_grasps", -1)),
                "num_yaws": int(request.get("moe_num_yaws", 36)),
                "z_offsets_cm": tuple(
                    request.get("moe_z_offsets_cm", (-8, -6, -4, -2, 0))
                ),
                "outlier_threshold": float(
                    request.get("moe_outlier_threshold", 0.014)
                ),
                "outlier_k": int(request.get("moe_outlier_k", 20)),
                "obb_mode": str(request.get("moe_obb_mode", "advanced")),
                "skip_obb_rule": str(request.get("moe_skip_obb_rule", "auto")),
                "obb_density": str(
                    request.get("moe_obb_density", "dense-topandside")
                ),
                "obb_position_spacing_m": float(
                    request.get("moe_obb_position_spacing_cm", 1.0)
                )
                / 100.0,
            }
            try:
                moe = run_graspmoe(point_cloud, self._sampler, **moe_params)
            except ValueError as exc:
                # e.g. suction gripper with no `width` field in its YAML --
                # see graspmoe._resolve_gripper_geometry.
                return {"error": f"graspmoe planner error: {exc}"}
            grasps_np = np.concatenate(
                [moe["grasps_diff"], moe["grasps_obb"]], axis=0
            ).astype(np.float32)
            conf_np = np.concatenate(
                [moe["scores_diff"], moe["scores_obb"]], axis=0
            ).astype(np.float32)
            branch_tags = ["diff"] * len(moe["grasps_diff"]) + ["obb"] * len(
                moe["grasps_obb"]
            )
            skipped_obb = bool(moe["skipped_obb"])
        else:
            params = {
                "grasp_threshold": float(request.get("grasp_threshold", -1.0)),
                "num_grasps": int(request.get("num_grasps", 200)),
                "topk_num_grasps": int(request.get("topk_num_grasps", -1)),
                "min_grasps": int(request.get("min_grasps", 40)),
                "max_tries": int(request.get("max_tries", 6)),
                "remove_outliers": bool(request.get("remove_outliers", True)),
            }
            grasps, grasp_conf = GraspGenSampler.run_inference(
                point_cloud, self._sampler, **params
            )
            grasps_np = (
                grasps.cpu().numpy().astype(np.float32)
                if len(grasps) > 0
                else np.empty((0, 4, 4), dtype=np.float32)
            )
            conf_np = (
                grasp_conf.cpu().numpy().astype(np.float32)
                if len(grasp_conf) > 0
                else np.empty((0,), dtype=np.float32)
            )
        infer_ms = (time.monotonic() - t0) * 1000

        if len(grasps_np) == 0:
            response = {
                "grasps": np.empty((0, 4, 4), dtype=np.float32),
                "confidences": np.empty((0,), dtype=np.float32),
                "num_grasps": 0,
                "timing": {"infer_ms": infer_ms},
                "planner": planner,
            }
            if branch_tags is not None:
                response["branch_tags"] = []
                response["skipped_obb"] = skipped_obb
            return response

        logger.info(
            "Inferred %d grasps (planner=%s) in %.1f ms (conf range %.3f - %.3f)",
            len(grasps_np),
            planner,
            infer_ms,
            conf_np.min(),
            conf_np.max(),
        )

        timing = {"infer_ms": infer_ms}
        response = {
            "grasps": grasps_np,
            "confidences": conf_np,
            "num_grasps": len(grasps_np),
            "timing": timing,
            "planner": planner,
        }
        if branch_tags is not None:
            response["branch_tags"] = branch_tags
            response["skipped_obb"] = skipped_obb

        # Optional collision filtering against a scene point cloud (table, other
        # objects, ...) with the target object's own points already excluded by the
        # caller. Opt-in: skipped entirely if the client doesn't send scene_point_cloud,
        # or if this gripper's collision mesh failed to load at startup.
        scene_point_cloud = request.get("scene_point_cloud")
        if scene_point_cloud is not None and self._gripper_surface_points is not None:
            scene_pc = np.asarray(scene_point_cloud, dtype=np.float32)
            max_scene_points = int(request.get("max_scene_points", 8192))
            if len(scene_pc) > max_scene_points:
                idx = np.random.choice(len(scene_pc), max_scene_points, replace=False)
                scene_pc = scene_pc[idx]

            collision_threshold = float(request.get("collision_threshold", 0.02))

            t1 = time.monotonic()
            collision_free_mask = filter_colliding_grasps_fast(
                scene_pc=scene_pc,
                grasp_poses=grasps_np,
                collision_threshold=collision_threshold,
                gripper_surface_points=self._gripper_surface_points,
            )
            collision_filter_ms = (time.monotonic() - t1) * 1000

            n_before = len(grasps_np)
            grasps_np = grasps_np[collision_free_mask]
            conf_np = conf_np[collision_free_mask]
            if branch_tags is not None:
                branch_tags = [
                    tag for tag, keep in zip(branch_tags, collision_free_mask) if keep
                ]
                response["branch_tags"] = branch_tags

            logger.info(
                "Collision filter: %d/%d grasps collision-free (thr=%.3fm, scene_pts=%d) in %.1f ms",
                len(grasps_np),
                n_before,
                collision_threshold,
                len(scene_pc),
                collision_filter_ms,
            )

            timing["collision_filter_ms"] = collision_filter_ms
            response["grasps"] = grasps_np
            response["confidences"] = conf_np
            response["num_grasps"] = len(grasps_np)
            response["num_grasps_before_collision_filter"] = n_before

        return response
