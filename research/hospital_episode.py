"""Pure extraction of official episode 16; the runner authors USD RotateZ degrees."""
import hashlib
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
BENCHMARK = ROOT/"DynaNav/configs/benchmark_full.yaml"
RUNNER = ROOT/"DynaNav/benchmark.py"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def official_yaw_radians(degrees):
    # benchmark.py:_spawn_robot passes start_yaw directly to AddRotateZOp().Set.
    return math.radians(float(degrees))


def official_episode(episode_id="episode_16"):
    if episode_id == "blind_door27":
        from research.blind_corner import custom_source
        return custom_source()
    if episode_id not in {"episode_16", "episode_21"}:
        raise ValueError("Unsupported research episode")
    episode = yaml.safe_load(BENCHMARK.read_text())[episode_id]
    return {"episode_id": episode_id, "benchmark_source": str(BENCHMARK.relative_to(ROOT)),
            "benchmark_sha256": digest(BENCHMARK), "runner_source": str(RUNNER.relative_to(ROOT)),
            "runner_sha256": digest(RUNNER), "source_convention": "USD RotateZ degrees",
            "resolved_yaw_radians": official_yaw_radians(episode["start_yaw"]), "episode": episode}


def validate_config(cfg):
    source = official_episode(cfg.get("blind_corner_episode_id", "episode_16"))
    e = source["episode"]
    if (cfg["official_episode"] != source or cfg["scene"]["mode"] != "official_usd"
            or cfg["scene"]["usd"] != e["scene"] or cfg["scene"]["goal"] != e["goal"]
            or cfg["robot"]["start_position"][:2] != e["start"][:2]
            or cfg["robot"]["start_yaw"] != source["resolved_yaw_radians"]
            or cfg["instruction"] != e["instruction"] or e["num_people"] != 0
            or "pedestrian" in cfg or "obstacles" in cfg["scene"]):
        raise ValueError("Research config does not preserve the selected official episode")
    return source
