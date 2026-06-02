"""Canonical launch entry point for the data collection service."""
from __future__ import annotations
from pathlib import Path
import os, sys, shutil
ROOT = Path(os.environ.get("DATA_COLLECTION_SERVICE_ROOT", Path.cwd()))
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from core.config import load_session_config

def _find_python():
    """Find a Python 3.10 interpreter (ROS2 Humble requires 3.10).
    Set PYTHON_BIN env var to override auto-detection."""
    for candidate in [
        os.environ.get("PYTHON_BIN", ""),
        "python3.10",
        "/usr/bin/python3.10",
        "python3",
    ]:
        if candidate and shutil.which(candidate):
            return candidate
    return "python3"

def generate_launch_description():
    python_bin = _find_python()
    pythonpath = os.pathsep.join(e for e in [str(ROOT), os.environ.get("PYTHONPATH","")] if e)
    default_cfg = str(ROOT / "config/session/session_vr_ik_robot_button_control.yaml")
    proc = ExecuteProcess(
        cmd=[python_bin,"-m","core.runtime.ros2_collection_node",
             "--session-config", LaunchConfiguration("session_config"),
             "--output-dir", LaunchConfiguration("output_dir")],
        cwd=str(ROOT), additional_env={"PYTHONPATH": pythonpath}, output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("session_config", default_value=default_cfg, description="Path to session YAML"),
        DeclareLaunchArgument("output_dir", default_value="", description="Output directory for episodes"),
        LogInfo(msg="Starting data collection service"),
        OpaqueFunction(function=_validate_config),
        LogInfo(msg="Launching collection node"),
        proc,
    ])

def _validate_config(context):
    cfg_path = Path(LaunchConfiguration("session_config").perform(context)).expanduser().resolve()
    load_session_config(cfg_path)
    return [SetLaunchConfiguration("session_config", str(cfg_path))]
