
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include <cppbktree.hpp>


#define CHECK( x, y ) \
if ( !( (x) == (y) ) ) { \
    std::cerr << "Check at line " << __LINE__ << " failed! (" << (x) << " != " << (y) << ")\n"; \
    std::exit( 1 ); \
}


[[nodiscard]] inline std::chrono::time_point<std::chrono::high_resolution_clock>
now()
{
    return std::chrono::high_resolution_clock::now();
}


template<typename T0, typename T1>
[[nodiscard]] double
duration( const T0& t0,
          const T1& t1 = now() )
{
    return std::chrono::duration<double>( t1 - t0 ).count();
}


template<typename T0>
[[nodiscard]] double
duration( const T0& t0 )
{
    return duration( t0, now() );
}


using BKTree = CppBKTree<std::vector<std::uint8_t>, size_t>;
using Values = std::vector<std::vector<std::uint8_t> >;


void
checkCountBits()
{
    CHECK( countBits( 0 ), 0 );
    CHECK( countBits( 1 ), 1 );
    CHECK( countBits( 2 ), 1 );
    CHECK( countBits( 3 ), 2 );
    CHECK( countBits( 123 ), 6 );
    CHECK( countBits( std::numeric_limits<int>::max() ), std::numeric_limits<int>::digits );
}


void
checkHammingDistance()
{
    CHECK( hammingDistance( { 0x00 }, { 0x00 } ), 0 );
    CHECK( hammingDistance( { 0xFF }, { 0x00 } ), 8 );
    CHECK( hammingDistance( { 0xFF }, { 0x77 } ), 2 );
    CHECK( hammingDistance( { 0xFF, 0xFF, 0xFF, 0xFF }, { 0x00, 0x00, 0x00, 0x00 } ), 32 );
    CHECK( hammingDistance( { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
                            { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ), 64 );
    CHECK( hammingDistance( { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
                            { 0x00, 0x10, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00 } ), 60 );
}


void
checkBKTree()
{
    /* Check empty constructor and destructor */
    {
        BKTree tree( hammingDistance );
    }

    /* Single element checks */
    CHECK( BKTree( hammingDistance, Values{ { 0x00 } } ).find( { 0x0F }, 0 ).empty(), true );
    CHECK( BKTree( hammingDistance, Values{ { 0x00 } } ).find( { 0x0F }, 3 ).empty(), true );
    CHECK( BKTree( hammingDistance, Values{ { 0x01 } } ).find( { 0x0F }, 2 ).empty(), true );
    CHECK( BKTree( hammingDistance, Values{ { 0xFF } } ).find( { 0x0F }, 3 ).empty(), true );

    /* Check for identical matches */
    CHECK( BKTree( hammingDistance, Values{ { 0x00 } } ).find( { 0x00 }, 0 ), std::vector<size_t>{ 0 } );
    CHECK( BKTree( hammingDistance, Values{ { 0x01 } } ).find( { 0x01 }, 0 ), std::vector<size_t>{ 0 } );
    CHECK( BKTree( hammingDistance, Values{ { 0xFF } } ).find( { 0xFF }, 0 ), std::vector<size_t>{ 0 } );

    /* Check for near matches */
    CHECK( BKTree( hammingDistance, Values{ { 0x00 } } ).find( { 0x0F }, 4 ), std::vector<size_t>{ 0 } );
    CHECK( BKTree( hammingDistance, Values{ { 0x01 } } ).find( { 0x0F }, 3 ), std::vector<size_t>{ 0 } );
    CHECK( BKTree( hammingDistance, Values{ { 0xFF } } ).find( { 0x0F }, 4 ), std::vector<size_t>{ 0 } );

    CHECK( BKTree( hammingDistance, Values{ { 0x00 } } ).find( { 0x0F }, 10000 ), std::vector<size_t>{ 0 } );
    CHECK( BKTree( hammingDistance, Values{ { 0x01 } } ).find( { 0x0F }, 10000 ), std::vector<size_t>{ 0 } );
    CHECK( BKTree( hammingDistance, Values{ { 0xFF } } ).find( { 0x0F }, 10000 ), std::vector<size_t>{ 0 } );

    /* Check some more complex example */
    {
        /**
         *  0: 0000
         *  4: 0100
         *  5: 0101
         * 14: 1110
         * 15: 1111
         */
        BKTree tree( hammingDistance, Values{ { 0 }, { 4 }, { 5 }, { 14 } } );
        CHECK( tree.size(), 4 );
        tree.add( std::vector<uint8_t>{ 15 } );
        CHECK( tree.size(), 5 )

        /* Distance 1 */
        {
            auto resultIndexes = tree.find( std::vector<uint8_t>{ 13 } /* 1101 */, 1 );
            std::sort( resultIndexes.begin(), resultIndexes.end() );
            std::vector<size_t> expectedResult = { 2 /* 5 is the 2nd element added */,
                                                   4 /* 15 is the 4th element added */ };
            CHECK( resultIndexes, expectedResult );
        }

        /* Distance 2 */
        {
            auto resultIndexes = tree.find( std::vector<uint8_t>{ 13 } /* 1101 */, 2 );
            std::sort( resultIndexes.begin(), resultIndexes.end() );
            std::vector<size_t> expectedResult = { 1, 2, 3, 4 };
            CHECK( resultIndexes, expectedResult );
        }

        /* If the specified threshold distance is large enough, all elements should be returned! */
        {
            auto resultIndexes = tree.find( std::vector<uint8_t>{ 13 } /* 1101 */,
                                            std::numeric_limits<BKTree::DistanceType>::max() );
            std::sort( resultIndexes.begin(), resultIndexes.end() );
            std::vector<size_t> expectedResult( tree.size() );
            std::iota( expectedResult.begin(), expectedResult.end(), 0 );
            CHECK( resultIndexes, expectedResult );
        }
    }

    /* Check statistics */
    {
        BKTree tree( hammingDistance, Values{ { 5 }, { 7 }, { 6 }, { 0 }, { 13 }, { 3 }, { 9 }, { 10 } } );
        const auto stats = tree.statistics();
        CHECK( stats.nodeCount               , 8 )
        CHECK( stats.leafCount               , 4 )
        CHECK( stats.valueCount              , 8 )
        CHECK( stats.averageChildCountPerNode, 7. / 4. )
        CHECK( stats.maxDepth                , 4 )
        CHECK( stats.minChildrenPerNode      , 1 )
        CHECK( stats.maxChildrenPerNode      , 3 )
        CHECK( stats.duplicateCount          , 0 )
        CHECK( stats.valueBitCount           , 8 )
    }

    /* Do some generic tests by comparing BKTree find with the results of a linear search. */
}


void
benchmarkHammingLookup()
{
    const auto t0 = now();

    std::random_device randomDevice;
    std::default_random_engine randomEngine( randomDevice() );
    std::uniform_int_distribution<uint64_t> distribution( 0 );

    std::vector<uint64_t> hashes( 100'000'000 );
    for ( auto& h : hashes ) {
        h = distribution( randomEngine );
    }

    const auto t1 = now();
    std::cerr << "Generating random numbers took " << duration( t0, t1 ) << " s\n";

    const uint64_t needle( 0x1234'5678'90AB'CDEFULL );


    #pragma omp parallel for simd
    for ( size_t i = 0; i < hashes.size(); ++i ) {
        hashes[i] = std::bitset<64>( hashes[i] ^needle ).count() < 10;
    }

    const auto t2 = now();
    std::cerr << "Xoring took " << duration( t1, t2 ) << " s\n";

    std::cerr << "hashes[333]: " << hashes[333] << "\n";
}


int main()
{
    benchmarkHammingLookup();
    return 0;

    checkCountBits();
    checkHammingDistance();
    checkBKTree();
    return 0;
}
