from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ropose_datagrabber'],
    package_dir={'': 'pysrc'}
)

setup(**d)
