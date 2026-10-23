# edge_engine/setup.py
from setuptools import setup, Extension
import pybind11

ext_modules = [
    Extension(
        "motor_pid",
        ["motor_pid.cpp"],
        include_dirs=[pybind11.get_include()],
        language="c++",
        extra_compile_args=["-std=c++17", "-O3"],
    ),
]

setup(
    name="motor_pid",
    version="1.0.0",
    author="MECH-MINDS",
    description="C++17 PID & L298N Actuation Module",
    ext_modules=ext_modules,
)