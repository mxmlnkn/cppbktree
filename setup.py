#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from setuptools import setup
from setuptools.extension import Extension

buildCython = '--cython' in sys.argv

extensions = [
    Extension(
        name               = 'cppbktree',
        sources            = [ 'cppbktree/cppbktree.pyx' if buildCython
                               else 'cppbktree/cppbktree.cpp' ],
        include_dirs       = [ '.' ],
        language           = 'c++',
        extra_compile_args = [ '-std=c++11', '-O3', '-DNDEBUG' ],
    ),
]

if buildCython:
    from Cython.Build import cythonize
    extensions = cythonize( extensions, compiler_directives = { 'language_level' : '3' } )
    del sys.argv[sys.argv.index( '--cython' )]

scriptPath = os.path.abspath( os.path.dirname( __file__ ) )
with open( os.path.join( scriptPath, 'README.md' ), encoding = 'utf-8' ) as file:
    readmeContents = file.read()

setup(
    name             = 'cppbktree',
    version          = '0.0.1',

    description      = 'C++ Implementation of a Burkhard-Keller Tree (BK-Tree)',
    url              = 'https://github.com/mxmlnkn/cppbktree',
    author           = 'Maximilian Knespel',
    author_email     = 'mxmlnkn@github.de',
    license          = 'MIT',
    classifiers      = [ 'License :: OSI Approved :: MIT License',
                         'Development Status :: 3 - Alpha',
                         'Operating System :: POSIX',
                         'Operating System :: Unix',
                         'Programming Language :: Python :: 3' ],

    long_description = readmeContents,
    long_description_content_type = 'text/markdown',

    py_modules       = [ 'cppbktree' ],
    ext_modules      = extensions
)
