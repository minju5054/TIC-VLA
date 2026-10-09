"""Small append-only validation records, not a hard-case scoring pipeline."""
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from research.control import validate_chunk


def clocks():
    return {"wall_time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns()}


def write_json(path, data):
    with Path(path).open("x") as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write("\n")


def provenance(repo):
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=repo, text=True)
    paths = sorted(list((repo/"research").glob("*.py")) + list((repo/"configs/research").glob("*.yaml"))
                   + [repo/"scripts/isaac6_python.sh", repo/"DynaNav/ticvla.py",
                      repo/"DynaNav/ticvla_vlm.py", repo/"DynaNav/behavior/nova_carter_test_ticvla.py"])
    return {"local_sha": git("rev-parse", "HEAD").strip(),
            "upstream_sha": git("rev-parse", "upstream/main").strip(),
            "git_status": git("status", "--short"),
            "diff_sha256": hashlib.sha256(git("diff", "HEAD").encode()).hexdigest(),
            "source_sha256": {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}


class RunRecords:
    def __init__(self, path, metadata):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/"raw/requests").mkdir(parents=True)
        (self.path/"diagnostics").mkdir()
        write_json(self.path/"metadata.json", metadata)
        self.ids = set()

    def request(self, request_id, chunk, metadata):
        if type(request_id) is not int or request_id <= 0 or request_id in self.ids:
            raise ValueError("Request ID must be positive and unique in this run")
        a = validate_chunk(chunk)
        event = {**metadata, "request_id": request_id,
                 "raw_action_shape": list(a.shape), "storage_dtype": str(a.dtype),
                 "native_axes": ["forward", "left"], "native_frame": "observation_body",
                 "native_units": "meters", "native_yaw": None}
        json.dumps(event, allow_nan=False) # Reject invalid metadata before reserving array.
        stem = self.path/"raw/requests"/f"request_{request_id:06d}"
        # Reserve the array with exclusive creation; never truncate an earlier run.
        with stem.with_suffix(".npy").open("xb") as f:
            np.save(f, a, allow_pickle=False)
        write_json(stem.with_suffix(".json"), event)
        self.ids.add(request_id)
