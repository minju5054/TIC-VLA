"""Isaac-only startup, optionally followed by unmodified DynaNav/API imports.

Run with scripts/isaac6_python.sh. This is a diagnostic, not an episode runner,
compatibility shim, model validator, or chunk collector. No assets are spawned.
"""

import argparse
import importlib
import inspect
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

_failures = 0


def report(stage, status, **details):
    global _failures
    if status == "FAIL":
        _failures += 1
    print("ISAAC6_PROBE " + json.dumps({"stage": stage, "status": status, **details}), flush=True)


def attempt(name, callback):
    try:
        result = callback()
        report(name, "PASS", detail=result)
        return result
    except Exception as exc:
        report(name, "FAIL", error=repr(exc))
        return None


def import_info(name):
    module = importlib.import_module(name)
    return {"version": getattr(module, "__version__", None), "file": getattr(module, "__file__", None)}


def audit(app):
    # SimulationApp already returned: never import TIC-VLA before the startup gate.
    repo = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo / "DynaNav"))
    for name in ["numpy", "torch", "torchvision", "transformers", "huggingface_hub",
                 "accelerate", "einops", "timm", "cv2", "matplotlib", "yaml"]:
        attempt("dependency:" + name, lambda name=name: import_info(name))

    def cuda_kernel():
        import torch
        x = torch.ones((16, 16), device="cuda")
        value = (x @ x)[0, 0].item()
        assert value == 16.0
        return {"device": torch.cuda.get_device_name(), "result": value}

    attempt("api:torch_cuda_kernel", cuda_kernel)

    attempt("B:DynaNav.ticvla", lambda: import_info("ticvla"))
    attempt("C:benchmark", lambda: import_info("benchmark"))
    attempt("D:NovaCarterBehavior", lambda: import_info("behavior.nova_carter_test_ticvla"))

    import omni.kit.app
    manager = omni.kit.app.get_app().get_extension_manager()
    root = Path(os.environ["ISAAC_PATH"])
    # Inspect only bundled extensions. Do not resolve/download missing extensions.
    for name in ["omni.kit.scripting", "omni.anim.people", "omni.isaac.sensor",
                 "omni.behavior.scripting.core", "isaacsim.robot.wheeled_robots",
                 "isaacsim.sensors.physics", "isaacsim.replicator.agent.core"]:
        paths = [p for directory in ["exts", "extsDeprecated", "extscache"]
                 for p in (root / directory).glob(name + "*")
                 if p.name == name or p.name.startswith(name + "-")]
        report("extension:" + name, "PASS" if paths else "FAIL", installed_paths=[str(p) for p in paths])
        if paths:
            def enable(name=name):
                if not manager.set_extension_enabled_immediate(name, True):
                    raise RuntimeError("Extension manager rejected " + name)
                return True
            attempt("enable:" + name, enable)
    app.update()

    for name in ["omni.kit.scripting", "omni.anim.people.settings", "omni.behavior.scripting.core",
                 "omni.metropolis.utils.carb_util", "robot_navigation_manager",
                 "isaacsim.core.utils.stage", "isaacsim.core.utils.prims",
                 "isaacsim.core.api", "isaacsim.sensors.physics", "omni.replicator.core"]:
        attempt("api:" + name, lambda name=name: import_info(name))

    def signatures():
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        return {"DifferentialController": str(inspect.signature(DifferentialController)),
                "forward": str(inspect.signature(DifferentialController.forward)),
                "WheeledRobot": str(inspect.signature(WheeledRobot)),
                "apply_wheel_actions": str(inspect.signature(WheeledRobot.apply_wheel_actions))}

    attempt("api:wheel_signatures", signatures)

    def agent_api():
        from isaacsim.replicator.agent.core.simulation import SimulationManager
        methods = ["load_config_file", "register_set_up_simulation_done_callback",
                   "set_up_simulation_from_config_file", "setup_simulation"]
        result = {name: hasattr(SimulationManager, name) for name in methods}
        report("api:SimulationManager.methods", "PASS" if all(result.values()) else "FAIL", methods=result)

    attempt("api:SimulationManager.import", agent_api)

    def config_preflight():
        import yaml
        from benchmark import BenchmarkRunner
        from isaacsim.replicator.agent.core.configuration.models.root import RootConfig
        config = yaml.safe_load((repo / "DynaNav/configs/benchmark_example.yaml").read_text())
        # Invoke the unchanged upstream template method with temporary diagnostic
        # output paths. Do not generate commands, migrate the schema, or load assets.
        with tempfile.TemporaryDirectory(prefix="ticvla-isaac6-config-") as directory:
            runner = SimpleNamespace(config_file_path=str(Path(directory) / "benchmark.yaml"),
                                     _benchmark_config=config, episode_name="episode_hospital_smoke")
            path = BenchmarkRunner._create_episode_config(runner, config[runner.episode_name])
            generated = yaml.safe_load(Path(path).read_text())
            report("api:official_config_input", "PASS", config=generated)
            RootConfig.model_validate(generated)
        return True

    attempt("api:official_config_schema", config_preflight)
    # These are independent import checks, not a substitute for a scene run.
    for stage in ["E:scene", "F:robot", "G:RGB", "H:model", "I:checkpoint", "J:prediction", "K:controller"]:
        report(stage, "NOT ATTEMPTED", reason="Diagnostic stops at import/API preflight; see compatibility report.")
    return 1 if _failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", action="store_true", help="Check imports and installed APIs after startup")
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 640, "height": 480})
    print("ISAAC6_SIMULATION_APP_READY", flush=True)
    code = 0
    try:
        if args.audit:
            code = audit(app)
    except Exception:
        code = 1
        raise
    finally:
        report("shutdown_requested", "PASS", exit_code=code)
        # Isaac 6 fast shutdown terminates the process; a post-close print is unreliable.
        app.close(exit_code=code)
    return code


if __name__ == "__main__":
    sys.exit(main())
