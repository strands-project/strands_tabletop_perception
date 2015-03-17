from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['object_search_action'],
    scripts=['scripts/object_search_server.py'],
    package_dir={'': 'src'}
)

setup(**d)
