import os
import toml
import pathlib

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__)))

if os.path.exists(os.path.abspath(os.path.join(PROJECT_ROOT, 'configs'))):
    CONFIGS_ROOT = os.path.abspath(os.path.join(PROJECT_ROOT, 'configs'))
else:
    raise FileNotFoundError("Configs directory not found in PROJECT_ROOT or PACKAGE_ROOT.")

if os.path.exists(os.path.abspath(os.path.join(PROJECT_ROOT, 'assets'))):
    ASSETS_ROOT = os.path.abspath(os.path.join(PROJECT_ROOT, 'assets'))
else:
    raise FileNotFoundError("Assets directory not found in PROJECT_ROOT or ASSETS_ROOT.")

PROGRAM_SETTINGS = None
with open(os.path.join(PACKAGE_ROOT, "settings.toml"), "r") as f:
    PROGRAM_SETTINGS = toml.load(f)

try:
    from autolife_robot_sdk import GLOBAL_VARS, reload_sdk_constants
    GLOBAL_VARS.ACTIVE_ROBOT_VERSION = PROGRAM_SETTINGS["sdk_settings"]["active_robot_version"]
    reload_sdk_constants()
except ImportError:
    raise ImportError("Please install the autolife_robot_sdk package to use this module.")
