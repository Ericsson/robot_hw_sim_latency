## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['figment_ariac', 'figment_ariac.gprt'],
    package_dir={'': 'src'},
    scripts=['bin/scheduler_plan',],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg'])

setup(**setup_args)
