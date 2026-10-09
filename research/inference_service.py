"""Official DynaNav TICVLA inference in the existing Isaac 5 Python, no SimulationApp."""
import argparse
from collections import deque
import hashlib
import importlib.util
import importlib.metadata
from pathlib import Path
import socket
import sys
import traceback

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from research.ipc import send, receive
from research.records import clocks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fd", type=int, required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    sock = socket.socket(fileno=args.fd)
    model = None
    try:
        import cv2
        import numpy as np
        import torch
        import yaml
        from research.control import rotation_wxyz, validate_chunk
        cfg = yaml.safe_load(Path(args.config).read_text())
        torch.manual_seed(cfg["seed"])
        np.random.seed(cfg["seed"])
        base, checkpoint = Path(cfg["inference"]["base_model"]), Path(cfg["inference"]["checkpoint"])
        hashes = {}
        for key, file, expected in [
            ("base", base/"model.safetensors", "a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484"),
            ("checkpoint", checkpoint, "376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036")]:
            with file.open("rb") as f:
                hashes[key] = hashlib.file_digest(f, "sha256").hexdigest()
            if hashes[key] != expected:
                raise RuntimeError("Artifact hash mismatch: " + str(file))
        sys.path.insert(0, str(REPO/"DynaNav"))
        spec = importlib.util.spec_from_file_location("official_dynanav_model", REPO/"DynaNav/ticvla.py")
        runtime = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runtime)
        model = runtime.TICVLA(model_path=str(base), device=cfg["inference"]["device"])
        state_dict = torch.load(checkpoint, map_location="cpu")["state_dict"]
        if not all(k.startswith("model.") for k in state_dict):
            raise RuntimeError("Unexpected checkpoint prefix")
        model.load_state_dict({k[len("model."):]: v for k, v in state_dict.items()}, strict=True)
        del state_dict
        model.eval()
        torch.cuda.synchronize()
        if any(k.startswith("isaacsim.simulation_app") for k in sys.modules):
            raise RuntimeError("Inference service unexpectedly imported SimulationApp")
        send(sock, {"type": "ready", "model_source": runtime.__file__, "strict_checkpoint": True,
                    "python": sys.version, "executable": sys.executable,
                    "isaac_version": (Path(cfg["inference"]["python"]).parent/"VERSION").read_text().strip(),
                    "torch": torch.__version__, "numpy": np.__version__,
                    "transformers": importlib.metadata.version("transformers"),
                    "torchvision": importlib.metadata.version("torchvision"),
                    "device": torch.cuda.get_device_name(), "artifact_sha256": hashes,
                    "simulation_app_started": False, **clocks()})
        directory = Path(args.run_dir)/"runtime_images"
        directory.mkdir(exist_ok=False)
        images = []
        starts = deque(maxlen=2)
        last_id = 0
        while True:
            request, encoded = receive(sock)
            if request.get("type") == "shutdown":
                break
            rid = request["request_id"]
            if not isinstance(rid, int) or rid <= last_id:
                raise ValueError("Requests must be sequential unique IDs")
            last_id = rid
            image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
            expected_resolution = cfg["camera"]["resolution"]
            if image is None or image.shape != (expected_resolution[1], expected_resolution[0], 3):
                raise ValueError("Invalid RGB PNG payload")
            # Match the official behavior's cv2 JPEG buffer, then use original preprocessing.
            success, jpeg = cv2.imencode(".jpg", image)
            if not success:
                raise RuntimeError("JPEG encoding failed")
            path = directory/f"request_{rid:06d}.jpg"
            with path.open("xb") as f:
                f.write(jpeg.tobytes())
            obs = request["observation"]
            images.append((obs["sim_time"], str(path), rid))
            sampled = []
            for multiplier in [3, 2, 1, 0]:
                offset = multiplier*cfg["inference"]["image_history_interval_seconds"]
                candidates = [item for item in images if item[0] <= obs["sim_time"]-offset+1e-8]
                if candidates and candidates[-1] not in sampled:
                    sampled.append(candidates[-1])
            delta = np.zeros(3)
            delay = 0.0
            reference = starts[0] if starts else None
            if reference is not None:
                delta = rotation_wxyz(reference["quaternion_wxyz"]).T @ (np.array(obs["position"])-reference["position"])
                delay = obs["sim_time"]-reference["sim_time"]
            # Preserve Nova Carter's explicit lateral velocity negation, but not for displacement.
            velocity = obs["linear_velocity_body_flu"]
            robot_state = [velocity[0], -velocity[1], velocity[2], obs["angular_velocity_body_flu"][2], float(delta[0]), float(delta[1])]
            pose = {"position": obs["position"], "quaternion": obs["quaternion_wxyz"],
                    "rotation_matrix": rotation_wxyz(obs["quaternion_wxyz"]).tolist()}
            start = clocks()
            with torch.no_grad():
                text, action, gen_step, kv_available, gen_pose = model.predict_async(
                    image_paths=[x[1] for x in sampled], instruction=request["instruction"],
                    robot_state=torch.tensor(robot_state, dtype=torch.float32), current_step=obs["tick"],
                    current_robot_pose=pose, previous_waypoints_text=request["previous_waypoints_text"],
                    time_delay=delay, robot_type="wheeled robot")
            torch.cuda.synchronize()
            if tuple(action.shape) != (1, 30, 2):
                raise ValueError(f"Unexpected native tensor shape {tuple(action.shape)}")
            raw = validate_chunk(action[0].float().cpu().numpy())
            ready = clocks()
            if gen_step is not None:
                if gen_step != obs["tick"] or gen_pose is None:
                    raise RuntimeError("Cannot associate reasoning start with this observation")
                starts.append({"request_id": rid, "tick": gen_step, "sim_time": obs["sim_time"],
                               "position": np.asarray(gen_pose["position"]).tolist(),
                               "quaternion_wxyz": np.asarray(gen_pose["quaternion"]).tolist()})
            send(sock, {"type": "prediction", "request_id": rid, "raw_action": raw.tolist(),
                "native_tensor_shape": list(action.shape), "native_dtype": str(action.dtype),
                "inference_start": start, "ready": ready, "robot_state_input": robot_state,
                "delay_seconds_input": delay, "delay_reference": reference,
                "sampled_image_request_ids": [x[2] for x in sampled], "reasoning_text": text,
                "reasoning_new_start_step": gen_step, "reasoning_completion_poll_step": model._kv_cache_completion_step,
                "reasoning_exact_worker_ready_wall_time_ns": None, "kv_cache_available": bool(kv_available)})
    except Exception as exc:
        traceback.print_exc()
        try:
            send(sock, {"type": "error", "error": repr(exc)})
        except OSError:
            pass
        raise
    finally:
        if model is not None:
            model.cleanup()
        sock.close()


if __name__ == "__main__":
    main()
