from ament_uncrustify.main import main


# usage (from pami directory): pytest test/test_uncrustify.py
def test_uncrustify():
    cfg_file = "../robot/ros_ws/src/herminebot_bringup/config/ament_code_style.cfg"  # ROS2 project uncrustify config
    rc = main(argv=[f"-c{cfg_file}"])
    assert rc == 0, "Found uncrustify errors"
