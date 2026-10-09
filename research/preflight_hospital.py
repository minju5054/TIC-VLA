"""Model-free official Hospital episode placement and RGB preflight."""
import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    import yaml
    from research.records import RunRecords, write_json, provenance
    from research.hospital_episode import validate_config
    cfg = yaml.safe_load(Path(args.config).read_text()); validate_config(cfg)
    records = RunRecords(args.output_dir, {"mode": "hospital_preflight_no_model", "config": cfg, **provenance(ROOT)})
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 640, "height": 480})
    sim, code = None, 1
    try:
        from research.sim import Simulation
        sim = Simulation(cfg, records, "static")
        _, state, rgb = sim.capture("hospital_start.png")
        write_json(records.path/"summary.json", {"status": "PASS", "model_calls": 0,
            "observation": state, "rgb_path": str(rgb), "visual_review_required": True})
        print("HOSPITAL_PREFLIGHT_PASS", json.dumps(state), flush=True)
        code = 0
    except Exception as exc:
        traceback.print_exc(); write_json(records.path/"failure.json", {"error": repr(exc)})
    finally:
        if sim:
            sim.close()
        app.close(exit_code=code)


if __name__ == "__main__":
    main()
