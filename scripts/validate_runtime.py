"""Import/config checks and optional official DynaNav model construction.

Run with Isaac Sim 5.0.0 python.sh after sourcing .env.testing.local.
This does not generate action chunks or implement a research logger.
"""
import argparse
import importlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--construct", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    print("upstream_sha", subprocess.check_output(["git", "rev-parse", "upstream/main"], cwd=root, text=True).strip())
    print("local_sha", subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip())
    print("git_status", subprocess.check_output(["git", "status", "--short"], cwd=root, text=True))
    print("python", sys.executable, sys.version)
    isaac_root = Path(os.environ["ISAAC_SIM_ROOT"])
    assert Path(sys.executable).resolve().is_relative_to(isaac_root.resolve()), "Not the selected Isaac Python"
    version = (isaac_root / "VERSION").read_text().strip()
    assert version == "5.0.0" or version.startswith(("5.0.0-", "5.0.0+")), version
    assert Path(os.environ["ISAAC_SIM_PYTHON"]).is_file()
    print("isaac_version", version)
    for name in ("numpy", "torch", "torchvision", "transformers", "huggingface_hub", "accelerate", "einops", "timm", "cv2", "matplotlib", "yaml", "isaacsim"):
        module = importlib.import_module(name)
        print("IMPORT PASS", name, getattr(module, "__version__", "n/a"))
    from isaacsim import SimulationApp  # noqa: F401; import only, no app launch
    assert callable(SimulationApp), "Isaac failed to expose SimulationApp"
    import numpy as np
    import torch
    import yaml

    assert int(np.__version__.split(".")[0]) < 2
    assert torch.cuda.is_available(), "CUDA unavailable"
    print("cuda", torch.version.cuda, "devices", torch.cuda.device_count(), "name", torch.cuda.get_device_name(0))
    # Exercise the actual installed CUDA binary on this GPU, not only detection.
    value = torch.ones((16, 16), device="cuda")
    assert (value @ value).sum().item() == 4096
    torch.cuda.synchronize()
    print("CUDA kernel PASS")

    sys.path.insert(0, str(root))
    try:
        package = importlib.import_module("ticvla.models.ticvla")
        print("package_model", package.__file__)
    except ModuleNotFoundError as exc:
        if exc.name != "pytorch_lightning":
            raise
        print("PACKAGE IMPORT FAIL: upstream package eagerly imports training-only pytorch_lightning; not installed for DynaNav")
        # This optional package check must not prevent independent validation of
        # the actual sibling-module DynaNav inference path.
    # Avoid confusing the installed package with DynaNav's same-name module.
    sys.path.insert(0, str(root / "DynaNav"))
    spec = importlib.util.spec_from_file_location("dynanav_ticvla_runtime", root / "DynaNav/ticvla.py")
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    print("runtime_model", runtime.__file__)
    import benchmark
    config = benchmark._expand_env_value(yaml.safe_load((root / "DynaNav/configs/benchmark_example.yaml").read_text()))
    episodes = {k: v for k, v in config.items() if isinstance(v, dict)}
    assert len(episodes) == 4
    for name, episode in episodes.items():
        scene = episode["scene"]
        assert "${" not in scene
        if not scene.startswith("https://"):
            assert Path(scene).is_file(), scene
        print("CONFIG PASS", name, scene)

    base = Path(os.environ["TICVLA_BASE_MODEL_PATH"])
    checkpoint = Path(os.environ["TICVLA_CHECKPOINT_PATH"])
    assert (base / "config.json").is_file() and (base / "model.safetensors").is_file()
    assert checkpoint.is_file()
    print("ARTIFACT paths PASS", base, checkpoint, checkpoint.stat().st_size)
    if not args.construct:
        print("STATIC RUNTIME PASS; construction not requested")
        return

    torch.cuda.reset_peak_memory_stats()
    model = runtime.TICVLA(model_path=str(base), device="cuda:0")
    try:
        print("CONSTRUCTION PASS", "allocated_MiB", torch.cuda.memory_allocated() / 2**20)
        # Match Nova Carter's released checkpoint load and strictness exactly.
        state_dict = torch.load(checkpoint, map_location="cpu")["state_dict"]
        state_dict = {key[len("model."):]: value for key, value in state_dict.items()}
        print("checkpoint_action_embedding", tuple(state_dict["action_expert.action_chunk_embed.weight"].shape))
        print("checkpoint_output_weight", tuple(state_dict["action_expert.mlp.2.weight"].shape))
        model.load_state_dict(state_dict, strict=True)
        model.eval()
        torch.cuda.synchronize()
        print("CHECKPOINT strict load PASS")
        print("cuda_allocated_MiB", torch.cuda.memory_allocated() / 2**20)
        print("cuda_reserved_MiB", torch.cuda.memory_reserved() / 2**20)
        print("cuda_peak_allocated_MiB", torch.cuda.max_memory_allocated() / 2**20)
    finally:
        model.cleanup()


if __name__ == "__main__":
    main()
