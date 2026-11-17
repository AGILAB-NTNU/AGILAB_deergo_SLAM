"""Select a usable CUDA MPPI plugin, or keep the portable CPU configuration."""

import argparse
import copy
import ctypes
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

CPU_PLUGIN = "nav2_mppi_controller::MPPIController"
GPU_PLUGIN = "mppi_controller_cuda::MPPIControllerCUDA"


def probe_driver():
    """Read CUDA device capability without requiring a toolkit or ROS node."""
    if os.environ.get("CUDA_VISIBLE_DEVICES") in ("", "-1", "NoDevFiles"):
        raise RuntimeError("CUDA devices are hidden by CUDA_VISIBLE_DEVICES")
    driver = ctypes.CDLL("libcuda.so.1")
    if driver.cuInit(0) != 0:
        raise RuntimeError("CUDA driver initialization failed")
    count = ctypes.c_int()
    if driver.cuDeviceGetCount(ctypes.byref(count)) != 0 or count.value < 1:
        raise RuntimeError("No CUDA device is available")
    major, minor = ctypes.c_int(), ctypes.c_int()
    if (
        driver.cuDeviceComputeCapability(ctypes.byref(major), ctypes.byref(minor), 0)
        != 0
    ):
        raise RuntimeError("Cannot read CUDA compute capability")
    return {"available": True, "architecture": f"{major.value}{minor.value}"}


def probe_gpu():
    """Execute a kernel in the actual plugin before selecting it for Nav2."""
    from ament_index_python.packages import get_package_prefix

    result = probe_driver()
    prefix = Path(get_package_prefix("mppi_controller_cuda"))
    library = ctypes.CDLL(str(prefix / "lib/libmppi_controller_cuda.so"))
    for name in ("deergo_mppi_profile_abi", "deergo_mppi_batch_size"):
        function = getattr(library, name)
        function.restype = ctypes.c_int
        function.argtypes = []
    if library.deergo_mppi_profile_abi() != 1:
        raise RuntimeError("CUDA MPPI requires the shared-profile deployment patch")
    result["profile_abi"] = 1
    result["batch_size"] = library.deergo_mppi_batch_size()
    kernel_probe = library.deergo_mppi_cuda_probe
    kernel_probe.restype = ctypes.c_int
    kernel_probe.argtypes = []
    status = kernel_probe()
    if status != 0:
        raise RuntimeError(f"CUDA MPPI kernel probe failed: status {status}")
    return result


def gpu_status():
    """Contain driver/plugin failures and CUDA contexts in a short subprocess."""
    try:
        process = subprocess.run(
            [sys.executable, "-s", "-m", __name__, "--probe-gpu"],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        if process.returncode != 0:
            reason = process.stderr.strip() or process.stdout.strip()
            return {"available": False, "reason": reason or "GPU probe failed"}
        return json.loads(process.stdout)
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        return {"available": False, "reason": str(error)}


def cuda_parameters(profile):
    """Change only the execution plugin; both plugins read the same parameters."""
    result = copy.deepcopy(profile)
    if profile.get("plugin") == CPU_PLUGIN:
        result["plugin"] = GPU_PLUGIN
    else:
        result["primary_controller"] = GPU_PLUGIN
    return result


def profile_support_error(profile, report):
    """Never silently use a different sampling count or unsupported scoring."""
    if report.get("profile_abi") != 1:
        return "CUDA plugin needs the shared-profile deployment patch"
    if profile["batch_size"] != report.get("batch_size"):
        return (
            "CUDA batch_size differs; rebuild with scripts/install_mppi.sh or use CPU"
        )
    if not 1 <= profile["time_steps"] <= 100:
        return "CUDA supports time_steps between 1 and 100"
    if profile["motion_model"] != "DiffDrive":
        return "Shared CUDA profile currently supports DiffDrive"
    supported = {
        "ConstraintCritic",
        "CostCritic",
        "GoalCritic",
        "GoalAngleCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathAngleCritic",
        "PreferForwardCritic",
        "TwirlingCritic",
    }
    for critic in profile["critics"]:
        if critic not in supported:
            return f"CUDA does not implement {critic}; preserving CPU scoring"
        if profile.get(critic, {}).get("cost_power", 1) != 1:
            return "CUDA shared scoring currently requires cost_power=1"
    if profile.get("PathAlignCritic", {}).get("use_path_orientations", False):
        return "CUDA shared PathAlignCritic requires use_path_orientations=false"
    if profile.get("visualize", False):
        return "Full trajectory visualization uses the native CPU implementation"
    if not profile.get("PathAngleCritic", {}).get("forward_preference", True):
        return "CUDA shared PathAngleCritic requires forward_preference=true"
    return None


def select_profile(params_file, backend="auto", status=None):
    """Return (path, selected backend, reason); DWB profiles pass through."""
    if backend not in ("auto", "cpu", "gpu"):
        raise ValueError("mppi_backend must be auto, cpu or gpu")
    path = Path(params_file).expanduser().resolve(strict=True)
    config = yaml.safe_load(path.read_text())
    follow = config["controller_server"]["ros__parameters"]["FollowPath"]
    if CPU_PLUGIN not in (follow.get("plugin"), follow.get("primary_controller")):
        return str(path), "unchanged", "Existing controller profile"
    if backend == "cpu":
        return str(path), "cpu", "CPU explicitly selected"
    report = gpu_status() if status is None else status
    if report.get("available"):
        reason = profile_support_error(follow, report)
        if reason:
            report = {"available": False, "reason": reason}
    if not report.get("available"):
        reason = report.get("reason", "CUDA unavailable")
        if backend == "gpu":
            raise RuntimeError(f"GPU explicitly requested but unavailable: {reason}")
        return str(path), "cpu", f"Automatic CPU fallback: {reason}"
    config["controller_server"]["ros__parameters"]["FollowPath"] = cuda_parameters(
        follow
    )
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="deergo_mppi_gpu_", suffix=".yaml", delete=False
    ) as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
        selected_path = stream.name
    return selected_path, "gpu", "CUDA driver, plugin and kernel probe passed"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-driver", action="store_true")
    parser.add_argument("--probe-gpu", action="store_true")
    arguments = parser.parse_args()
    try:
        result = probe_driver() if arguments.probe_driver else probe_gpu()
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
