from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['xdot_k', 'match, manipulation'],
    package_dir={'': 'src'}
    )
setup(**d)