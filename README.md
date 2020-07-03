# C++ BK-Tree

[![PyPI version](https://badge.fury.io/py/cppbktree.svg)](https://badge.fury.io/py/cppbktree)
[![Downloads](https://pepy.tech/badge/cppbktree/month)](https://pepy.tech/project/cppbktree/month)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](http://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/mxmlnkn/cppbktree.svg?branch=master)](https://travis-ci.com/mxmlnkn/cppbktree)

This module provides a BK-Tree class written in C++ to hopefully better speed than preexisting pure-python solutions.


# Installation

You can simply install it from PyPI:
```
pip install cppbktree
```

# Usage

```python3
from cppbktree import BKTree

tree = BKTree( [ bytes( [ x ] ) for x in [0, 4, 5, 14] ] )
tree.add( bytes( [ 15 ] ) )
print( tree.find( 13, 1 ) )
```
