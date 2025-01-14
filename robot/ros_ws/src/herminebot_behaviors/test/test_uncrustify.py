from ament_index_python.packages import get_package_share_directory
from ament_uncrustify.main import main
import os
import pytest

@pytest.mark.linter
def test_uncrustify():
    cfg_file = os.path.join(get_package_share_directory("herminebot_bringup"), "config", "ament_code_style.cfg")
    rc = main(argv=[f"-c{cfg_file}"])
    assert rc == 0, "Found uncrustify errors"
