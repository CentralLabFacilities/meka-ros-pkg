from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
     packages=['meka_posture'],
     package_dir={'': 'src'}
)

setup(**d)

