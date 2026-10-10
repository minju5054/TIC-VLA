"""Pure source audit; official controller values are extracted, not inferred."""
import ast
from pathlib import Path
from research.hospital_episode import ROOT, digest, official_episode

BEHAVIOR = ROOT/"DynaNav/behavior/nova_carter_test_ticvla.py"
RUNNER = ROOT/"DynaNav/benchmark.py"


def source_audit(episode_id="episode_16"):
    tree = ast.parse(BEHAVIOR.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "NovaCarterTICVLA")
    names = ["_wheel_joints", "_wheel_radius", "_wheel_base", "_max_accel_lin", "_max_decel_lin",
             "_max_accel_ang", "_max_decel_ang", "_lin_deadband", "_ang_deadband"]
    constants = {n.targets[0].id: ast.literal_eval(n.value) for n in cls.body
                 if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) and n.targets[0].id in names}
    cameras = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
               and n.value.endswith("front_hawk/left/camera_left")]
    runner = ast.parse(RUNNER.read_text())
    assets = [n.value for n in ast.walk(runner) if isinstance(n, ast.Constant) and isinstance(n.value, str)
              and n.value.endswith("nova_carter_sensors.usd")]
    if len(set(assets)) != 1 or len(set(cameras)) != 1 or len(constants) != len(names):
        raise ValueError("Official Nova source changed; inspect before selecting asset/controller")
    return {"asset": assets[0], "front_camera": cameras[0], "constants": constants,
            "behavior_sha256": digest(BEHAVIOR), "runner_sha256": digest(RUNNER),
            "episode": official_episode(episode_id), "slew_note": "accel/decel chosen by signed target-current, exactly as upstream"}


def slew(cur, tgt, accel, decel, dt, deadband):
    """Exact nominal upstream _slew arithmetic; no recovery/backup policy."""
    dt = max(1e-4, float(dt))
    dv = tgt-cur
    limit = accel if dv > 0 else decel
    step = limit*dt
    cur = cur+step if dv > step else cur-step if dv < -step else tgt
    if abs(cur) < deadband and abs(tgt) < deadband:
        cur = 0.0
    return cur
