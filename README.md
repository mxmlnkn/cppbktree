# C++ BK-Tree

[![PyPI version](https://badge.fury.io/py/cppbktree.svg)](https://badge.fury.io/py/cppbktree)
[![Downloads](https://pepy.tech/badge/cppbktree/month)](https://pepy.tech/project/cppbktree/month)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](http://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/mxmlnkn/cppbktree.svg?branch=master)](https://travis-ci.com/mxmlnkn/cppbktree)

This module provides a BK-Tree class written in C++ to hopefully better speed than pure-python solutions.


# Installation

You can simply install it from PyPI:
```
pip install cppbktree
```

## From Source

```python3
python3 -m build . &&
python3 -m pip install dist/cppbktree-*.whl
```


# Usage

```python3
from cppbktree import BKTree

tree = BKTree( [ bytes( [ x ] ) for x in [0, 4, 5, 14] ] )
tree.add( bytes( [ 15 ] ) )
print( tree.find( 13, 1 ) )
```

Because of the Python/C++ interface, currently this BK-Tree is limited to only hamming distances of bytearrays.
Pull requests are welcome.

# Benchmarks

The benchmark consists of inserting a varying amount of random 64-bit elements into the tree and then querying a value at varying distances.
This is done five times to get a hint for the standard deviation as is plotted with error bars.

## Comparison pybktree vs. cppbktree 0.1.0 with 32 K element chunking for 64-bit elements

![Comparison pybktree cppbktree chunked](benchmark/results/compare-scalings-pybktree-cppbktree-chunked-8K.png) 

This is a benchmark for the specialized `BKTree64`, which operates on 64-bit values.
Doing the same with `BKTree` and byte vectors of size 8 to represent 64-bit values is roughly 4x slower
because of additional allocations and because of pointer-chasing slowing down the linear lookup.
A more intricate version could simply concatenate the element vectors into one consecutive memory chunk to avoid these performance issues.

| Operation | pybktree / s | cppbktree (8K chunks) / s | Speedup |
|-----------------------|:--------:|:--------:|:----:|
| Tree creation time    | 88.53    | 1.57     | 56
| Distance threshold 0  | 2.42e-04 | 1.81e-05 | 13
| Distance threshold 1  | 7.49e-04 | 1.11e-04 | 6.7
| Distance threshold 2  | 8.55e-03 | 4.46e-04 | 19
| Distance threshold 4  | 2.21e-01 | 3.35e-03 | 66
| Distance threshold 8  | 4.22e+00 | 1.55e-02 | 272
| Distance threshold 16 | 1.15e+01 | 3.00e-02 | 383


## Scaling Analysis

![Comparison pybktree cppbktree](benchmark/results/compare-scalings-pybktree-cppbktree.png)

In this log-log plot, it can be seen that the lookups and creations follow various sublinear power laws.
Inserting one element in a tree of depth $d$ should roughly take `O(log(d))` hamming distance evaluations.
Assuming an evenly distributed tree, the number of elements is given as `N=d^n` where `n` is the maximum distance the metric returns.
For the hamming distance, n is the number of bits of the hash.
Solving this for the depth, gives `d=log N / log n`.
If you are only interested in the dependence to `N`, then `log n` can be seen as a small constant factor.
Henceforth, the tree creation should follow `O(\sum_i^N \log i) = O( log( \product_i^N i )  ) = O(log(N!))`.
Using the Stirling's approximation for the faculty, we get `O(log(sqrt(N) N^N)) = O(log(N^(N+1/2))) = O(N log(N))`.
However, `log(N)` is a very slow growing function, so the tree creation looks almost a linear function.

Both, pybktree and cppbktree, have some jumps at roughly 1e4 elements but only cppbktree as a second jump at ~2e6 elements but only when looking up elements with distance <= 16.
I can't explain these jumps.
They almost look like memory caching effects.
Because of these jumps, the effective speedups for 10M elements varies quite a lot depending on the lookup distance.
Only the tree creation scaling is a very smooth curve except for some outliers for smaller runtimes.

Here are the fitted power laws to the curves from the plot:

| operation             | pybktree        | cppbktree
|-----------------------|:---------------:|:---------------:|
| Tree creation         | 1.12e-06 N^1.12 | 4.92e-07 N^1.05 |
| Distance threshold 0  | 2.04e-06 N^0.27 | 4.19e-07 N^0.27 |
| Distance threshold 1  | 2.06e-06 N^0.37 | 2.77e-07 N^0.38 |
| Distance threshold 2  | 1.68e-06 N^0.51 | 1.76e-07 N^0.54 |
| Distance threshold 4  | 1.36e-06 N^0.73 | 1.11e-07 N^0.74 |
| Distance threshold 8  | 1.10e-06 N^0.92 | 7.47e-08 N^0.94 |
| Distance threshold 16 | 1.05e-06 N^0.99 | 4.93e-08 N^1.05 |

And here are the timings and speedups for operations on a tree with 10 million 64-bit elements:

| Operation | pybktree / s | cppbktree / s | Speedup |
|-----------------------|:--------:|:--------:|:----:|
| Tree creation time    | 88.53    | 19.35    | 4.6  |
| Distance threshold 0  | 2.42e-04 | 2.38e-05 | 10.2 |
| Distance threshold 1  | 7.49e-04 | 1.55e-04 | 4.8  |
| Distance threshold 2  | 8.55e-03 | 1.61e-03 | 5.3  |
| Distance threshold 4  | 2.21e-01 | 3.73e-02 | 5.9  |
| Distance threshold 8  | 4.22     | 0.60     | 7.1  |
| Distance threshold 16 | 11.5     | 6.93     | 1.7  |

The speedups of cppbktree over pybktree vary between ~2 and 10.
For smaller trees, the speedups would be even better.
Only the tree creation time speedup is quite independent of the tree size at roughly 5.


## Comparison pybktree vs. vptree


![Comparison pybktree cppbktree](benchmark/results/compare-scalings-pybktree-vptree.png)

At least in this benchmark with only 64-bit hashes and a hamming distance as metric and at least with this pure python implementation of a VP-Tree, the results are quite disappointing.
The vptree module is almost always slower.
The lookups are actually quite similar to pybktree (meaning still slower than lookups with cppbktree) but the tree creation is a full magnitude slower.
For the 100k elements, this results in pybktree being 7.7 times faster than vptree.


## Comparison linear lookup vs. cppbktree

![Comparison pybktree cppbktree](benchmark/results/compare-scalings-cppbktree-linear-lookup.png)

This log-log comparison plot shows that a simple linear lookup can compete with a BK tree.
For exact and almost exact lookups, the BK tree can become faster but even for a distance of 2,
it requires more than 1 M elements of size 8 B to amortize.
For lookups with even larger distance, the simple linear lookup dominates for the whole tested range up to 80 MB of data!

The advantage of the linear lookup is better streaming behavior and less branching making it run faster on modern CPUs.
For batched lookup, the advantage might be even larger because whole batches can be looked up while the chunks of the haystack is still in CPU caches.
A mixture a BK tree and linear lookup, e.g., a BK tree with nodes with 1-10k elements, might be better over the whole range of test configurations.
