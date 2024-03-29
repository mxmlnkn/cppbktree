#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import pprint
import sys
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import scipy.stats
import tqdm
import cppbktree

try:
    import pybktree
except ImportError:
    pass

try:
    import vptree
except ImportError:
    pass


def benchmark_64_bit_hashes( element_counts, class_to_use, repeat_count = 10 ):
    """
    Returns a list of triples:
      - elements
      - tree creation time in seconds
      - lookup time for one element in seconds
    """
    timings = []
    for element_count in tqdm.tqdm( element_counts ):
        timing = [ element_count ]

        # Compare results to pybktree and print out statistics
        for distance in [ 0, 1, 2, 4, 8, 16 ]:
            elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )

            cpptree = class_to_use( elements )
            cppresults = sorted( elements[cpptree.find( 0, distance )] )
            print( "CppResults:", cppresults )

            if 'pybktree' in sys.modules:
                pytree = pybktree.BKTree( pybktree.hamming_distance, elements )
                pyresults = sorted( [ x[1] for x in pytree.find( np.uint64( 0 ), distance ) ] )
                print( "PyResults:", pyresults )

                assert np.all( pyresults == cppresults )


        runtimes = []
        for i in range( repeat_count ):
            # BKTree only takes bytearrays as values and internally assumes a hamming_distance for now
            # because it's trouble to specify arbitrary objects and metrics through the Cython interface.
            elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
            t0 = time.time()
            database = class_to_use( elements )
            t1 = time.time()
            runtimes.append( t1 - t0 )
        timing += [ np.mean( runtimes ), np.std( runtimes ) ]

        for distance in [ 0, 1, 2, 4, 8, 16 ]:
            runtimes = []
            for i in range( repeat_count ):
                elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
                database = class_to_use( elements )
                t0 = time.time()
                results = database.find( 0, distance )
                t1 = time.time()
                runtimes.append( t1 - t0 )
            timing += [ distance, np.mean( runtimes ), np.std( runtimes ) ]

        timings.append( timing )

    return timings

def benchmark_cppbktree( element_counts, repeat_count = 10 ):
    """
    Returns a list of triples:
      - elements
      - tree creation time in seconds
      - lookup time for one element in seconds
    """
    timings = []
    for element_count in tqdm.tqdm( element_counts ):
        timing = [ element_count ]

        # Compare results to pybktree and print out statistics
        for distance in [ 0, 1, 2, 4, 8, 16 ]:
            elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )

            byte_elements = [ int( x ).to_bytes( 8, 'big' ) for x in elements ]
            cpptree = cppbktree.BKTree( byte_elements )
            cppresults = sorted( elements[cpptree.find( ( 0 ).to_bytes( 8, 'big' ), distance )] )
            print( "CppResults:", cppresults )

            if 'pybktree' in sys.modules:
                pytree = pybktree.BKTree( pybktree.hamming_distance, elements )
                pyresults = sorted( [ x[1] for x in pytree.find( np.uint64( 0 ), distance ) ] )
                print( "PyResults:", pyresults )

                assert np.all( pyresults == cppresults )

            print( f"Tree statistics for {element_count} elements and distance <= {distance}: {cpptree.statistics()}" )
            print()


        runtimes = []
        for i in range( repeat_count ):
            # BKTree only takes bytearrays as values and internally assumes a hamming_distance for now
            # because it's trouble to specify arbitrary objects and metrics through the Cython interface.
            elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
            elements = [ int( x ).to_bytes( 8, 'big' ) for x in elements ]
            t0 = time.time()
            tree = cppbktree.BKTree( elements )
            t1 = time.time()
            runtimes.append( t1 - t0 )
        timing += [ np.mean( runtimes ), np.std( runtimes ) ]

        for distance in [ 0, 1, 2, 4, 8, 16 ]:
            runtimes = []
            for i in range( repeat_count ):
                elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
                elements = [ int( x ).to_bytes( 8, 'big' ) for x in elements ]
                tree = cppbktree.BKTree( elements )
                t0 = time.time()
                results = tree.find( ( 0 ).to_bytes( 8, 'big' ), distance )
                t1 = time.time()
                runtimes.append( t1 - t0 )
            timing += [ distance, np.mean( runtimes ), np.std( runtimes ) ]

        timings.append( timing )

    return timings

def benchmark_vptree( element_counts, repeat_count = 10 ):
    """
    Returns a list of triples:
      - elements
      - tree creation time in seconds
      - lookup time for one element in seconds
    """
    timings = []
    for element_count in tqdm.tqdm( element_counts ):
        timing = [ element_count ]

        runtimes = []
        for i in range( repeat_count ):
            elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
            t0 = time.time()
            tree = vptree.VPTree( elements, pybktree.hamming_distance )
            t1 = time.time()
            runtimes.append( t1 - t0 )
        timing += [ np.mean( runtimes ), np.std( runtimes ) ]

        for distance in [ 0, 1, 2, 4, 8, 16 ]:
            runtimes = []
            for i in range( repeat_count ):
                elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
                tree = vptree.VPTree( elements, pybktree.hamming_distance )
                t0 = time.time()
                results = tree.get_all_in_range( np.uint64( 0 ), distance )
                t1 = time.time()
                runtimes.append( t1 - t0 )
            timing += [ distance, np.mean( runtimes ), np.std( runtimes ) ]

        timings.append( timing )

    return timings

def benchmark_pybktree( element_counts, repeat_count = 10 ):
    """
    Returns a list of triples:
      - elements
      - tree creation time in seconds
      - lookup time for one element in seconds
    """
    timings = []
    for element_count in tqdm.tqdm( element_counts ):
        timing = [ element_count ]

        runtimes = []
        for i in range( repeat_count ):
            elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
            t0 = time.time()
            tree = pybktree.BKTree( pybktree.hamming_distance, elements )
            t1 = time.time()
            runtimes.append( t1 - t0 )
        timing += [ np.mean( runtimes ), np.std( runtimes ) ]

        for distance in [ 0, 1, 2, 4, 8, 16 ]:
            runtimes = []
            for i in range( repeat_count ):
                elements = np.random.randint( np.iinfo( np.uint64 ).max, size = element_count, dtype = np.uint64 )
                tree = pybktree.BKTree( pybktree.hamming_distance, elements )
                t0 = time.time()
                results = tree.find( item = np.uint64( 0 ), n = distance )
                t1 = time.time()
                runtimes.append( t1 - t0 )
            timing += [ distance, np.mean( runtimes ), np.std( runtimes ) ]

        timings.append( timing )

    return timings


def fit_power_law( x, y ):
    """
    In log-log space power laws becom linear laws. Consider the generic f(x) = a*x^b,
    in log-log space it would be log(f(x)) = log(a) + b*log(x).
    Returns the tuple (a,b), i.e., (factor, exponent).
    """
    slope, intercept, rvalue, pvalue, stderr = scipy.stats.linregress( np.log( x ), np.log( y ) )
    return np.exp( intercept ), slope


def plot_results_to_axis( ax, timings, linestyle = '-', plot_fits = True ):
    timings = np.array( timings )

    element_counts      = timings[:,0]
    creation_times_mean = timings[:,1]
    creation_times_std  = timings[:,2]

    colors = iter( plt.rcParams["axes.prop_cycle"].by_key()["color"] )


    custom_lines = []

    # Note that absolut +- errors become relative erros when log-scaled by using Gaussian error propagation
    relErrors = creation_times_std / creation_times_mean
    color = 'k'
    custom_lines.append( Line2D( [0], [0], color = color, marker = '+', label = "Tree creation" ) )
    ax.errorbar( element_counts, creation_times_mean,
                 [ creation_times_mean - creation_times_mean / ( 1 + relErrors ),
                   creation_times_mean * ( 1 + relErrors ) - creation_times_mean ],
                 marker = '+', linestyle = linestyle, color = color, capsize = 2, label = "Tree creation" )
    a,b = fit_power_law( element_counts, creation_times_mean )
    if plot_fits:
        ax.plot( element_counts, a * element_counts**b, color = '0.5', linestyle = '--' )
    print( f"Fitted creation time to {a} N**{b}" )

    for i in range( ( timings.shape[1] - 3 ) // 3 ):
        lookup_distances  = timings[:, 3 + 3*i+0]
        lookup_times_mean = timings[:, 3 + 3*i+1]
        lookup_times_std  = timings[:, 3 + 3*i+2]

        assert np.all( lookup_distances == lookup_distances[0] )

        relErrors = lookup_times_std / lookup_times_mean
        color = next( colors )
        label = "Lookup distance <= {}".format( lookup_distances[0] )
        custom_lines.append( Line2D( [0], [0], color = color, marker = '.', label = label ) )
        ax.errorbar( element_counts, lookup_times_mean,
                     [ lookup_times_mean - lookup_times_mean / ( 1 + relErrors ),
                       lookup_times_mean * ( 1 + relErrors ) - lookup_times_mean ],
                     marker = '.', linestyle = linestyle, color = color,
                     label = label, capsize = 2 )
        a,b = fit_power_law( element_counts, lookup_times_mean )
        if plot_fits:
            ax.plot( element_counts, a * element_counts**b, color = '0.5', linestyle = '--' )
        print( f"Fitted lookup for distance <= {int(lookup_distances[0])} time to {a:.2e} N**{b:.2f}" )

    if plot_fits:
        ax.plot( element_counts, np.log( element_counts ) / np.log( element_counts )[1] * creation_times_mean[1],
                 label = 'O(log N)', zorder = 20, color = 'k')

    return custom_lines

def plot_results( timings, export_name = None ):
    timings = np.array( timings )

    fig = plt.figure()

    ax = fig.add_subplot( 111,
        xlabel = "Number of Elements",
        ylabel = "Execution Time / s",
        xscale = 'log',
        yscale = 'log',
    )

    plot_results_to_axis( ax, timings )

    ax.legend( loc = 'best' )

    fig.tight_layout()

    if export_name:
        fig.savefig( export_name + '.pdf' )
        fig.savefig( export_name + '.png' )


def compare_scaling( data_files, export_name = None, print_numerical_comparison = True ):
    fig = plt.figure()

    ax = fig.add_subplot( 111,
        xlabel = "Number of Elements",
        ylabel = "Execution Time / s",
        xscale = 'log',
        yscale = 'log',
    )

    linestyles = [ '-', ':', '--', '-.' ]
    for data_file, linestyle in zip( data_files, linestyles ):
        legend_lines = plot_results_to_axis( ax, np.genfromtxt( data_file ), linestyle, plot_fits = False )
    for data_file, linestyle in zip( data_files, linestyles ):
        legend_lines.append( Line2D( [ 0 ], [ 0 ], color = '0.5', linestyle = linestyle, label = data_file.split( '/' )[-1] ) )

    ax.yaxis.set_major_locator( matplotlib.ticker.LogLocator( numticks = 20 ) )
    ax.yaxis.set_minor_locator( matplotlib.ticker.LogLocator( subs = 'all', numticks = 30 ) )

    #ax.grid()
    ax.legend( handles = legend_lines, loc = 'best' )

    fig.tight_layout()

    if export_name:
        fig.savefig( export_name + '.pdf' )
        fig.savefig( export_name + '.png' )
        print( f"Save image to: {export_name}.png" )

    if print_numerical_comparison and len( data_files ) == 2:
        # Give out some textual speedups for 1e7 elements
        data1 = np.genfromtxt( data_files[0] );
        data2 = np.genfromtxt( data_files[1] );
        element_count = data1[-1,0]
        assert element_count == data2[-1,0]
        print( f"Label              | {data_files[0]} | {data_files[1]} | speedup" )
        print( f"Tree creation time | {data1[-1,1]:.2f} | {data2[-1,1]:.2f} | {data1[-1,1] / data2[-1,1]:.2f}" )
        for i in range( ( data1.shape[1] - 3 ) // 3 ):
            print( f"Distance threshold {int(data1[0,3 + 3*i+0])} | {data1[-1,3 + 3*i+1]:.2e} | {data2[-1,3 + 3*i+1]:.2e} | {data1[-1,3 + 3*i+1] / data2[-1,3 + 3*i+1]:.1f}" )


if __name__ == '__main__':
    os.makedirs("results", exist_ok = True)

    MAX_MAGNITUDE = 7
    CHUNK_SIZE_IN_KILO_BINARY = 8
    element_sizes = np.unique( ( 10 ** np.linspace( 0, MAX_MAGNITUDE, 40 ) ).astype( np.int64 ) )

    if False:
        filePath = f"results/pybktree-scaling-1e{MAX_MAGNITUDE}.dat"
        if 'pybktree' in sys.modules and not os.path.exists( filePath ):
            timings = benchmark_pybktree( element_sizes )
            np.savetxt( filePath, timings )
        plot_results( np.genfromtxt( filePath ), export_name = filePath.replace( ".dat", "" ) )

        filePath = f"results/vpktree-scaling-1e{MAX_MAGNITUDE}.dat"
        if 'vptree' in sys.modules and not os.path.exists( filePath ):
            timings = benchmark_vptree( element_sizes )
            np.savetxt( filePath, timings )
        plot_results( np.genfromtxt( filePath ), export_name = filePath.replace( ".dat", "" ) )

        filePath = f"results/cppbktree-scaling-1e{MAX_MAGNITUDE}.dat"
        if not os.path.exists( filePath ):
            timings = benchmark_cppbktree( element_sizes )
            np.savetxt( filePath, timings )
        plot_results( np.genfromtxt( filePath ), export_name = filePath.replace( ".dat", "" ) )

        filePath = f"results/linear-lookup-scaling-1e{MAX_MAGNITUDE}.dat"
        if not os.path.exists( filePath ):
            timings = benchmark_64_bit_hashes( element_sizes, cppbktree.LinearLookup64 )
            np.savetxt( filePath, timings )
        plot_results( np.genfromtxt( filePath ), export_name = filePath.replace( ".dat", "" ) )

        filePath = f"results/cppbktree-chunked-{CHUNK_SIZE_IN_KILO_BINARY}K-scaling-1e{MAX_MAGNITUDE}.dat"
        if not os.path.exists( filePath ):
            timings = benchmark_64_bit_hashes( element_sizes,
                                               lambda x: cppbktree.BKTree64( x, CHUNK_SIZE_IN_KILO_BINARY * 1024 ) )
            np.savetxt( filePath, timings )
        plot_results( np.genfromtxt( filePath ), export_name = filePath.replace( ".dat", "" ) )

    #compare_scaling( [
    #    'results/pybktree-scaling-1e7.dat',
    #    'results/cppbktree-scaling-1e7.dat'
    #], 'results/compare-scalings-pybktree-cppbktree' )
    #compare_scaling( [ 'results/pybktree-scaling-1e5.dat', 'results/vptree-scaling-1e5.dat' ],
    #                 'results/compare-scalings-pybktree-vptree' )
    #compare_scaling( [
    #    f"results/linear-lookup-scaling-1e{MAX_MAGNITUDE}.dat",
    #    f"results/cppbktree-scaling-1e{MAX_MAGNITUDE}.dat",
    #], "results/compare-scalings-cppbktree-linear-lookup" )

    compare_scaling( [
        f"results/cppbktree-scaling-1e{MAX_MAGNITUDE}.dat",
        f"results/cppbktree-chunked-{CHUNK_SIZE_IN_KILO_BINARY}K-scaling-1e{MAX_MAGNITUDE}.dat",
    ], f"results/compare-scalings-cppbktree-vs-chunked-{CHUNK_SIZE_IN_KILO_BINARY}K" )

    compare_scaling( [
        f"results/pybktree-scaling-1e{MAX_MAGNITUDE}.dat",
        f"results/cppbktree-chunked-{CHUNK_SIZE_IN_KILO_BINARY}K-scaling-1e{MAX_MAGNITUDE}.dat",
    ], f"results/compare-scalings-pybktree-cppbktree-chunked-{CHUNK_SIZE_IN_KILO_BINARY}K" )

    plt.show()
