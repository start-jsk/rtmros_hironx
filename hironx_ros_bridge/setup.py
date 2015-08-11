## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['hironx_ros_bridge', 'hironx_ros_bridge.testutil', 'hironx_ros_bridge.hrpsys_315_1_9.hrpsys'],
    package_dir={'': 'src'})

setup(**setup_args)
