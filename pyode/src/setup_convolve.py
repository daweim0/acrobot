__author__ = 'David Smart'

import setuptools
import os
import numpy
from distutils.core import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize("convolve1.pyx"),
    include_dirs=[numpy.get_include()]
)

def make_setup_args():
    if os.name == 'nt':
        return dict(script_args=["--compiler=cygwin32 -w"])
    else:
        return dict(script_args=["-w"])
