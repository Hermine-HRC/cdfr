# This file serves to configure pytest before running tests

import sys

import hrc_utils


def pytest_configure():
    # Add the virtual environment to PYTHONPATH
    venv_path = hrc_utils.get_venv_site_packages_dir()
    if venv_path not in sys.path:
        sys.path.insert(0, venv_path)
