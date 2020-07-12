
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <vector>

#include <cppbktree.hpp>


#define CHECK( x, y ) \
if ( !( (x) == (y) ) ) { \
    std::cerr << "Check at line " << __LINE__ << " failed! (" << (x) << " != " << (y) << ")\n"; \
    std::exit( 1 ); \
}


using BKTree = CppBKTree<std::vector<std::uint8_t>, size_t>;
using Values = std::vector<std::vector<std::uint8_t> >;


std::mt19937 prng( 123456 );


std::vector<uint8_t>
createRandomVector( size_t size )
{
    std::uniform_int_distribution<uint8_t> uniformUInt8( 0 );

    std::vector<uint8_t> result( size );
    for ( auto& x : result ) {
        x = uniformUInt8( prng );
    }
    return result;
}


std::vector<std::vector<uint8_t> >
createRandomVectorVector( size_t vectorCount,
                          size_t vectorSize )
{
    std::vector<std::vector<uint8_t> > result( vectorCount );
    for ( auto& x : result ) {
        x = createRandomVector( vectorSize );
    }
    return result;
}


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

    /* Check saving and loading */
    const Values samples = { { 5 }, { 7 }, { 6 }, { 0 }, { 13 }, { 3 }, { 9 }, { 10 } };
    for ( size_t i = 0; i < samples.size(); ++i ) {
        auto subsamples = samples;
        subsamples.resize( i );
        std::stringstream buffer;

        BKTree tree( hammingDistance, subsamples );
        tree.serialize( buffer );

        BKTree loadedTree( hammingDistance );
        loadedTree.deserialize( buffer );

        CHECK( tree, loadedTree );
    }

    {
        std::stringstream buffer;

        BKTree tree( hammingDistance, createRandomVectorVector( 256, 2 ) );
        tree.serialize( buffer );

        BKTree loadedTree( hammingDistance );
        loadedTree.deserialize( buffer );

        CHECK( tree, loadedTree );
    }
}


int main()
{
    checkCountBits();
    checkHammingDistance();
    checkBKTree();
    return 0;
}
