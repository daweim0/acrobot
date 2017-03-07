__author__ = 'David Smart'

import setuptools
import os
import numpy
from distutils.core import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize("lookup_table_hopper_helper.pyx"),
    include_dirs=[numpy.get_include()], requires=['numpy']
)

def make_setup_args():
    if os.name == 'nt':
        return dict(script_args=["--compiler=cygwin32 -w"])
    else:
        return dict(script_args=["-w"])
