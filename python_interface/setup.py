import setuptools
from distutils.core import setup
from distutils.util import get_platform

with open("README.md", "r") as fh:
    long_description = fh.read()

platform = get_platform()
print(platform)

extensions = []

INSTALL_REQUIRES = [
    "rospy",
    "numpy",
]

setup(
    name="ros_python_interface",
    version="0.0.1",
    author="Zhuochen Liu",
    author_email="liuzhuoc@usc.edu",
    description="A python interface for ROS.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.6.*",
    install_requires=INSTALL_REQUIRES,
    packages=setuptools.find_packages(),
    ext_modules=extensions,
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
