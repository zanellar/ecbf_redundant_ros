"""Install the Python simulation and ROS orchestration package."""

from setuptools import find_packages, setup


setup(
    name="ecbf-redundant",
    version="0.2.0",
    description="Energy-limited control of a redundant Franka manipulator",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.10",
    install_requires=["numpy>=1.23", "PyYAML>=6.0"],
    extras_require={
        "simulation": ["mujoco>=3.0", "scipy>=1.10", "matplotlib>=3.7"],
        "test": ["pytest>=7.0"],
    },
)
