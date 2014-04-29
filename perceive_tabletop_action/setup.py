
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['perceive_tabletop_action'],
    scripts=['scripts/perceive_tabletop_action_server.py', 'scripts/perceive_tabletop_action_client.py', 'scripts/register_task.py'],
    package_dir={'': 'src'}
)

setup(**d)
