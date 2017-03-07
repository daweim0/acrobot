#!/bin/sh

cython lookup_table_hopper_helper.pyx
python setup.py build_ext --inplace
echo done
/usr/bin/python2.7 interpolation_consistancy_checker.py
