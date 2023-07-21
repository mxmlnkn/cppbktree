
#include <AlignedAllocator.hpp>
#include <TestHelpers.hpp>

#include <algorithm>
#include <bitset>
#include <chrono>
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
        CHECK( stats.nodeCount               , 1 )
        CHECK( stats.leafCount               , 1 )
        CHECK( stats.valueCount              , 8 )
        //CHECK( stats.averageChildCountPerNode, 7. / 4. )
        CHECK( stats.maxDepth                , 1 )
        CHECK( stats.minChildrenPerNode      , 0 )
        CHECK( stats.maxChildrenPerNode      , 0 )
        //CHECK( stats.duplicateCount          , 0 )
        CHECK( stats.valueBitCount           , 8 )
    }

    /* Do some generic tests by comparing BKTree find with the results of a linear search. */
}


#ifdef __AVX2__
alignas( sizeof( __m128i ) ) static constexpr auto BIT_COUNT_LUT =
    [] ()
    {
        /* Only the lowest 4 bits are used to query the lookup table. */
        alignas( 16 ) std::array<uint8_t, 16> result{};
        for ( size_t i = 0; i < result.size(); ++i ) {
            /* Counting bits set, Brian Kernighan's way. std::bitset::count is not constexpr. */
            for ( auto value = i; value > 0; ++result[i] ) {
                value &= value - 1U;  // clear least significant bit set. */
            }
        }
        return result;
    }();


[[nodiscard]] std::vector<uint8_t>
countDifferingBits8( const AlignedVector<uint8_t>& haystack,
                     const uint8_t                 needle )
{
    std::vector<uint8_t> counts( haystack.size() );

    /* Broadcast value to SIMD vector entries for fast xor-ing. May generate vpbroadcastq.
     * @see https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html */
    const auto needleRepeated = _mm256_set1_epi8( needle );

    /* Duplicate 128-bit LUT to 256-bit so that it works with the peculiar 256-bit shuffle version. */
    const auto bitCountLUT128 =
        _mm_load_si128( reinterpret_cast<const __m128i*>( BIT_COUNT_LUT.data() ) );  // 128 b = 16 B LUT size
    const auto bitCountLUT = _mm256_set_m128i( bitCountLUT128, bitCountLUT128 );

    const auto LOW_NIBBLES_SET = _mm256_set1_epi8( 0x0F );

    static constexpr size_t SIMD_SIZE = sizeof( __m256i ) / sizeof( haystack[0] );  // 32 B and 4 x 64-bit values!
    for ( size_t i = 0; i + SIMD_SIZE - 1 < haystack.size(); i += SIMD_SIZE ) {
        const auto values = _mm256_load_si256( reinterpret_cast<const __m256i*>( haystack.data() + i ) );
        const auto tested = values ^ needleRepeated;
        /* Count bits per byte using two LUT lookups, one for each nibble. The shuffle is 128-bit based, which makes
         * sense because it can hold LUTs for exactly 4-bit indexes. However, this means that the 256-bit version
         * is not what it seems, instead it is like like the 128-bit version with two different LUTs and index vectors.
         * So, if the same LUT is to be used for all indexes in the 256 vector, then the 128-bit LUT needs to be
         * duplicated to 256-bit instead of simply casting it up to 256-bit and filling the high bits with zero!
         * Note that it doesn't matter what exact shift is done because it only affects the high bits, which will
         * be zeroed out anyway. It would have been nice if there was a right shift working on 8-bit values in order
         * to avoid the additional masking operation, but that does not seem to be available with AVX, the smallest
         * shifts on 16-bit lanes. */
        const auto bitCounts8 = _mm256_shuffle_epi8( bitCountLUT, tested & LOW_NIBBLES_SET ) +
                                _mm256_shuffle_epi8( bitCountLUT, ( tested >> 4U ) & LOW_NIBBLES_SET );

        std::memcpy( counts.data() + i, reinterpret_cast<const uint8_t*>( &bitCounts8 ), sizeof( bitCounts8 ) );
    }

    return counts;
}


std::ostream&
operator<<( std::ostream&  out,
            const __m256i& values )
{
    std::array<uint8_t, sizeof( __m256i )> copy{};
    std::memcpy( copy.data(), &values, copy.size() );

    std::ios ioState( nullptr );
    ioState.copyfmt( out );

    out << "0x" << std::hex;
    for ( size_t i = 0; i < copy.size(); ++i ) {
        if ( ( i % 8 == 0 ) && ( i > 0 ) ) {
            out << " ";
        }
        out << " " << std::setfill( '0' ) << std::setw( 2 ) << static_cast<uint16_t>( copy[i] );
    }
    out.copyfmt( ioState );
    return out;
}


[[nodiscard]] std::vector<uint8_t>
countDifferingBits16( const AlignedVector<uint16_t>& haystack,
                      const uint16_t                 needle )
{
    std::vector<uint8_t> counts( haystack.size() );

    /* Broadcast value to SIMD vector entries for fast xor-ing. May generate vpbroadcastq.
     * @see https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html */
    const auto needleRepeated = _mm256_set1_epi16( needle );
    //std::cerr << "Needle repeated: " << needleRepeated << "\n";

    /* Duplicate 128-bit LUT to 256-bit so that it works with the peculiar 256-bit shuffle version. */
    const auto bitCountLUT128 =
        _mm_load_si128( reinterpret_cast<const __m128i*>( BIT_COUNT_LUT.data() ) );  // 128 b = 16 B LUT size
    const auto bitCountLUT = _mm256_set_m128i( bitCountLUT128, bitCountLUT128 );

    const auto LOW_NIBBLES_SET = _mm256_set1_epi8( 0x0F );

    static constexpr auto SELECT_REDUCED_BYTES_ARRAY =
        [] ()
        {
            std::array<uint8_t, 16> result{};
            for ( auto& x : result ) {
                x = 0xFF;
            }
            for ( size_t i = 0; i < result.size() / 2; ++i ) {
                const auto index = result.size() - 1 - i;
                result[index] = result.size() - 2 - 2 * i;
            }
            return result;
        } ();

    /* Duplicate 128-bit LUT to 256-bit so that it works with the peculiar 256-bit shuffle version. */
    const auto SELECT_REDUCED_BYTES_128 =
        _mm_loadu_si128( reinterpret_cast<const __m128i*>( SELECT_REDUCED_BYTES_ARRAY.data() ) );
    const auto SELECT_REDUCED_BYTES = _mm256_set_m128i( SELECT_REDUCED_BYTES_128, SELECT_REDUCED_BYTES_128 );
    //std::cerr  << "SELECT_REDUCED_BYTES: " << SELECT_REDUCED_BYTES << "\n";

    static constexpr size_t SIMD_ELEMENTS = sizeof( __m256i ) / sizeof( haystack[0] );  // 32 B and 4 x 64-bit values!
    for ( size_t i = 0; i + SIMD_ELEMENTS - 1 < haystack.size(); i += SIMD_ELEMENTS ) {
        //std::cerr << "Iteration " << i << "\n";
        const auto values = _mm256_load_si256( reinterpret_cast<const __m256i*>( haystack.data() + i ) );
        const auto tested = values ^ needleRepeated;
        //std::cerr << "values     : " << values << "\n";
        //std::cerr << "tested     : " << tested << "\n";
        /* Count bits per byte using two LUT lookups, one for each nibble. The shuffle is 128-bit based, which makes
         * sense because it can hold LUTs for exactly 4-bit indexes. However, this means that the 256-bit version
         * is not what it seems, instead it is like like the 128-bit version with two different LUTs and index vectors.
         * So, if the same LUT is to be used for all indexes in the 256 vector, then the 128-bit LUT needs to be
         * duplicated to 256-bit instead of simply casting it up to 256-bit and filling the high bits with zero!
         * Note that it doesn't matter what exact shift is done because it only affects the high bits, which will
         * be zeroed out anyway. It would have been nice if there was a right shift working on 8-bit values in order
         * to avoid the additional masking operation, but that does not seem to be available with AVX, the smallest
         * shifts on 16-bit lanes. */
        const auto bitCounts8 = _mm256_shuffle_epi8( bitCountLUT, tested & LOW_NIBBLES_SET ) +
                                _mm256_shuffle_epi8( bitCountLUT, ( tested >> 4U ) & LOW_NIBBLES_SET );
        //std::cerr << "bitCounts8 : " << bitCounts8 << "\n";

        /**
         * Sum neighboring counts:
         * +---+---+---+---+
         * | 4 | 3 | 2 | 1 |
         * +---+---+---+---+
         *   +   +   +   +
         * +---+---+---+---+
         * | 0 | 4 | 3 | 2 |
         * +---+---+---+---+
         *         =
         * +---+---+---+---+
         * | 4 | 7 | 5 | 3 |
         * +---+---+---+---+
         *       ^       ^
         *  Actual values of interest
         */
        //std::cerr << "shifted 4  : " << ( bitCounts8 >> 4U ) << "\n";
        //std::cerr << "shifted 8  : " << ( bitCounts8 >> 8U ) << "\n";
        //std::cerr << "shifted 4  : " << _mm256_srli_epi32( bitCounts8, 4U ) << "\n";
        //std::cerr << "shifted 8  : " << _mm256_srli_epi32( bitCounts8, 8U ) << "\n";
        //std::cerr << "shifted 1  : " << _mm256_srli_epi16( bitCounts8, 1U ) << "\n";
        //std::cerr << "shifted 4  : " << _mm256_srli_epi16( bitCounts8, 4U ) << "\n";
        //std::cerr << "shifted 8  : " << _mm256_srli_epi16( bitCounts8, 8U ) << "\n";
        const auto bitCounts16 = bitCounts8 + _mm256_srli_epi16( bitCounts8, 8U );
        //std::cerr << "bitCounts16: " << bitCounts16 << "\n";

        /* Move bytes with counts for consecutive bits to the least significant positions and cast down. */
        const auto result = _mm256_shuffle_epi8( bitCounts16, SELECT_REDUCED_BYTES );

        //std::cerr << "Result     : " << result << "\n";

        std::memcpy( counts.data() + i,
                     reinterpret_cast<const uint8_t*>( &result ) + sizeof( result ) / 2 - SIMD_ELEMENTS / 2,
                     SIMD_ELEMENTS / 2 );
        std::memcpy( counts.data() + i + SIMD_ELEMENTS / 2,
                     reinterpret_cast<const uint8_t*>( &result ) + sizeof( result ) - SIMD_ELEMENTS / 2,
                     SIMD_ELEMENTS / 2 );
    }

    return counts;
}


[[nodiscard]] std::vector<size_t>
findClose16( const AlignedVector<uint16_t>& haystack,
             const uint16_t                 needle,
             const uint8_t                  distance )
{
    std::vector<uint8_t> counts( haystack.size() );

    /* Broadcast value to SIMD vector entries for fast xor-ing. May generate vpbroadcastq.
     * @see https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html */
    const auto needleRepeated = _mm256_set1_epi16( needle );
    /* Add 1 to distance because there is only == or > for 16-bit integer AVX2 instructinos. */
    const auto distanceRepeated = _mm256_set1_epi16( distance == std::numeric_limits<uint8_t>::max()
                                                     ? distance : distance + 1 );

    /* Duplicate 128-bit LUT to 256-bit so that it works with the peculiar 256-bit shuffle version. */
    const auto bitCountLUT128 =
        _mm_load_si128( reinterpret_cast<const __m128i*>( BIT_COUNT_LUT.data() ) );  // 128 b = 16 B LUT size
    const auto bitCountLUT = _mm256_set_m128i( bitCountLUT128, bitCountLUT128 );

    const auto LOW_NIBBLES_SET = _mm256_set1_epi8( 0x0F );
    const auto LOW_BYTES_SET = _mm256_set1_epi16( 0xFF );

    std::vector<size_t> matches;

    static constexpr size_t SIMD_ELEMENTS = sizeof( __m256i ) / sizeof( haystack[0] );  // 32 B and 4 x 64-bit values!
    for ( size_t i = 0; i + SIMD_ELEMENTS - 1 < haystack.size(); i += SIMD_ELEMENTS ) {
        const auto values = _mm256_load_si256( reinterpret_cast<const __m256i*>( haystack.data() + i ) );
        const auto tested = values ^ needleRepeated;
        /* Count bits per byte using two LUT lookups, one for each nibble. The shuffle is 128-bit based, which makes
         * sense because it can hold LUTs for exactly 4-bit indexes. However, this means that the 256-bit version
         * is not what it seems, instead it is like like the 128-bit version with two different LUTs and index vectors.
         * So, if the same LUT is to be used for all indexes in the 256 vector, then the 128-bit LUT needs to be
         * duplicated to 256-bit instead of simply casting it up to 256-bit and filling the high bits with zero!
         * Note that it doesn't matter what exact shift is done because it only affects the high bits, which will
         * be zeroed out anyway. It would have been nice if there was a right shift working on 8-bit values in order
         * to avoid the additional masking operation, but that does not seem to be available with AVX, the smallest
         * shifts on 16-bit lanes. */
        const auto bitCounts8 = _mm256_shuffle_epi8( bitCountLUT, tested & LOW_NIBBLES_SET ) +
                                _mm256_shuffle_epi8( bitCountLUT, ( tested >> 4U ) & LOW_NIBBLES_SET );

        /**
         * Sum neighboring counts:
         * +---+---+---+---+
         * | 4 | 3 | 2 | 1 |
         * +---+---+---+---+
         *   +   +   +   +
         * +---+---+---+---+
         * | 0 | 4 | 3 | 2 |
         * +---+---+---+---+
         *         =
         * +---+---+---+---+
         * | 4 | 7 | 5 | 3 |
         * +---+---+---+---+
         *       ^       ^
         *  Actual values of interest
         */
        const auto bitCounts16 = ( bitCounts8 & LOW_BYTES_SET ) + _mm256_srli_epi16( bitCounts8, 8U );

        /* Move bytes with counts for consecutive bits to the least significant positions and cast down. */
        auto result = static_cast<uint32_t>( _mm256_movemask_epi8( _mm256_cmpgt_epi16( distanceRepeated, bitCounts16 ) ) );
        std::cerr << "Result     : " << std::hex << result << std::dec << "\n";
        for ( size_t j = 0; result != 0; result >>= sizeof( needle ), ++j ) {
            if ( ( result & 1U ) > 0 ) {
                matches.push_back( i + j );
            }
        }

        std::cerr << "Result     : " << std::hex << result << std::dec << "\n";
    }

    return matches;
}


[[nodiscard]] std::vector<size_t>
findClose64( const AlignedVector<uint64_t>& haystack,
             const uint64_t                 needle,
             const uint8_t                  distance )
{
    /* Broadcast value to SIMD vector entries for fast xor-ing. May generate vpbroadcastq.
     * @see https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html */
    const auto needleRepeated = _mm256_set1_epi64x( needle );
    /* Add 1 to distance because there is only == or > for 16-bit integer AVX2 instructinos. */
    const auto distanceRepeated = _mm256_set1_epi64x( distance == std::numeric_limits<uint8_t>::max()
                                                      ? distance : distance + 1 );
    //std::cerr << "Needle repeated: " << needleRepeated << "\n";

    /* Duplicate 128-bit LUT to 256-bit so that it works with the peculiar 256-bit shuffle version. */
    const auto bitCountLUT128 =
        _mm_load_si128( reinterpret_cast<const __m128i*>( BIT_COUNT_LUT.data() ) );  // 128 b = 16 B LUT size
    const auto bitCountLUT = _mm256_set_m128i( bitCountLUT128, bitCountLUT128 );

    const auto LOW_NIBBLES_SET = _mm256_set1_epi8( 0x0F );
    const auto LOW_BYTES_SET = _mm256_set1_epi16( 0xFF );
    //std::cerr << "low bytes  : " << LOW_BYTES_SET << "\n";

    std::vector<size_t> matches;

    static constexpr size_t SIMD_ELEMENTS = sizeof( __m256i ) / sizeof( haystack[0] );  // 32 B and 4 x 64-bit values!
    #pragma GCC unroll 8
    for ( size_t i = 0; i + SIMD_ELEMENTS - 1 < haystack.size(); i += SIMD_ELEMENTS ) {
        //std::cerr << "Iteration " << i << "\n";
        const auto values = _mm256_load_si256( reinterpret_cast<const __m256i*>( haystack.data() + i ) );
        const auto tested = values ^ needleRepeated;
        //std::cerr << "values     : " << values << "\n";
        //std::cerr << "tested     : " << tested << "\n";
        /* Count bits per byte using two LUT lookups, one for each nibble. The shuffle is 128-bit based, which makes
         * sense because it can hold LUTs for exactly 4-bit indexes. However, this means that the 256-bit version
         * is not what it seems, instead it is like like the 128-bit version with two different LUTs and index vectors.
         * So, if the same LUT is to be used for all indexes in the 256 vector, then the 128-bit LUT needs to be
         * duplicated to 256-bit instead of simply casting it up to 256-bit and filling the high bits with zero!
         * Note that it doesn't matter what exact shift is done because it only affects the high bits, which will
         * be zeroed out anyway. It would have been nice if there was a right shift working on 8-bit values in order
         * to avoid the additional masking operation, but that does not seem to be available with AVX, the smallest
         * shifts on 16-bit lanes. */
        const auto bitCounts8 = _mm256_shuffle_epi8( bitCountLUT, tested & LOW_NIBBLES_SET ) +
                                _mm256_shuffle_epi8( bitCountLUT, ( tested >> 4U ) & LOW_NIBBLES_SET );
        //std::cerr << "bitCounts8 : " << bitCounts8 << "\n";

        /**
         * Sum neighboring counts:
         * +---+---+---+---+
         * | 4 | 3 | 2 | 1 |
         * +---+---+---+---+
         *   +   +   +   +
         * +---+---+---+---+
         * | 0 | 4 | 3 | 2 |
         * +---+---+---+---+
         *         =
         * +---+---+---+---+
         * | 4 | 7 | 5 | 3 |
         * +---+---+---+---+
         *       ^       ^
         *  Actual values of interest
         */
        const auto bitCounts16 = ( bitCounts8  & LOW_BYTES_SET ) + _mm256_srli_epi64( bitCounts8 ,  8U );
        const auto bitCounts32 = ( bitCounts16 & LOW_BYTES_SET ) + _mm256_srli_epi64( bitCounts16, 16U );
        const auto bitCounts64 = ( bitCounts32 & LOW_BYTES_SET ) + _mm256_srli_epi64( bitCounts32, 32U );
        //std::cerr << "bitCounts16: " << bitCounts16 << "\n";
        //std::cerr << "bitCounts32: " << bitCounts32 << "\n";
        //std::cerr << "bitCounts64: " << bitCounts64 << "\n";

        /* Move bytes with counts for consecutive bits to the least significant positions and cast down. */
        auto result = static_cast<uint32_t>( _mm256_movemask_epi8( _mm256_cmpgt_epi16( distanceRepeated, bitCounts64 ) ) );
        for ( size_t j = 0; result != 0; result >>= sizeof( needle ), ++j ) {
            if ( ( result & 1U ) > 0 ) {
                matches.push_back( i + j );
            }
        }
    }

    return matches;
}


void
testCountDifferingBits8()
{
    REQUIRE_EQUAL( ( countDifferingBits8( AlignedVector<uint8_t>( 32, 0 ), 0x00 ) ), ( std::vector<uint8_t>( 32, 0 ) ) );
    REQUIRE_EQUAL( ( countDifferingBits8( AlignedVector<uint8_t>( 32, 0 ), 0x01 ) ), ( std::vector<uint8_t>( 32, 1 ) ) );
    REQUIRE_EQUAL( ( countDifferingBits8( AlignedVector<uint8_t>( 32, 1 ), 0x01 ) ), ( std::vector<uint8_t>( 32, 0 ) ) );
    REQUIRE_EQUAL( ( countDifferingBits8( AlignedVector<uint8_t>( 32, 0 ), 0x0F ) ), ( std::vector<uint8_t>( 32, 4 ) ) );
    REQUIRE_EQUAL( ( countDifferingBits8( AlignedVector<uint8_t>( 32, 0 ), 0xFF ) ), ( std::vector<uint8_t>( 32, 8 ) ) );

    {
        AlignedVector<uint8_t> haystack( 32, 0 );
        std::iota( haystack.begin(), haystack.end(), 0 );
        const auto result = countDifferingBits8( haystack, 15 );
        std::vector<uint8_t> expected = { 4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0,
                                          5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1 };
        REQUIRE_EQUAL( result, expected );
    }

    {
        AlignedVector<uint16_t> haystack( 16, 0 );
        std::iota( haystack.begin(), haystack.end(), 330 );

        const uint16_t needle = 333;

        const auto result = countDifferingBits16( haystack, needle );
        std::vector<uint8_t> expected = { 3, 2, 1, 0, 2, 1, 4, 3, 5, 4, 3, 2, 4, 3, 3, 2 };
        REQUIRE_EQUAL( result, expected );

        std::vector<uint8_t> expected2( haystack.size() );
        for ( size_t i = 0; i < expected2.size(); ++i ) {
            expected2[i] = std::bitset<16>( haystack[i] ^ needle ).count();
        }
        REQUIRE_EQUAL( result, expected2 );

        std::cerr << "Distance 0: " << findClose16( haystack, needle, 0 ) << "\n";
        std::cerr << "Distance 1: " << findClose16( haystack, needle, 1 ) << "\n";
        std::cerr << "Distance 2: " << findClose16( haystack, needle, 2 ) << "\n";
    }

    {
        AlignedVector<uint16_t> haystack( 16, 0 );
        std::iota( haystack.begin(), haystack.end(), 330 );

        const uint16_t needle = 0;

        const auto result = countDifferingBits16( haystack, needle );
        std::vector<uint8_t> expected = { 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5 };
        REQUIRE_EQUAL( result, expected );

        std::vector<uint8_t> expected2( haystack.size() );
        for ( size_t i = 0; i < expected2.size(); ++i ) {
            expected2[i] = std::bitset<16>( haystack[i] ^ needle ).count();
        }
        REQUIRE_EQUAL( result, expected2 );
    }

    {
        AlignedVector<uint64_t> haystack( 4, 0 );
        std::iota( haystack.begin(), haystack.end(), 330 );

        const uint64_t needle = 333;
        const size_t distance = 0;

        const auto result = findClose64( haystack, needle, distance );
        std::vector<size_t> expected = { 3 };
        REQUIRE_EQUAL( result, expected );

        std::vector<size_t> expected2;
        for ( size_t i = 0; i < haystack.size(); ++i ) {
            if ( std::bitset<64>( haystack[i] ^ needle ).count() <= distance ) {
                expected2.push_back( i );
            }
        }
        REQUIRE_EQUAL( result, expected2 );


        REQUIRE_EQUAL( ( findClose64( haystack, needle, 0 ) ), ( std::vector<size_t>( { 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 1 ) ), ( std::vector<size_t>( { 2, 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 2 ) ), ( std::vector<size_t>( { 1, 2, 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 3 ) ), ( std::vector<size_t>( { 0, 1, 2, 3 } ) ) );
    }

    {
        AlignedVector<uint64_t> haystack( 8, 0 );
        std::iota( haystack.begin(), haystack.end(), 330 );
        const uint64_t needle = 333;

        REQUIRE_EQUAL( ( findClose64( haystack, needle, 0 ) ), ( std::vector<size_t>( { 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 1 ) ), ( std::vector<size_t>( { 2, 3, 5 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 2 ) ), ( std::vector<size_t>( { 1, 2, 3, 4, 5 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 3 ) ), ( std::vector<size_t>( { 0, 1, 2, 3, 4, 5, 7 } ) ) );
    }

    {
        AlignedVector<uint64_t> haystack( 8, 0 );
        std::iota( haystack.begin(), haystack.end(), 0xFFFF'FF00ULL );
        const uint64_t needle = 0xFFFF'FF03ULL;

        REQUIRE_EQUAL( ( findClose64( haystack, needle, 0 ) ), ( std::vector<size_t>( { 3 } ) ) );
    }

    {
        AlignedVector<uint64_t> haystack = {
             4693705359351378771ULL,  // distance: 0
             6399443711166777426ULL,  // distance: 23
            17353886869887400783ULL,  // distance: 26
            16494262339385545536ULL,  // distance: 22
        };
        const uint64_t needle = 4693705359351378771ULL;

        REQUIRE_EQUAL( ( findClose64( haystack, needle, 0 ) ), ( std::vector<size_t>( { 0 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 21 ) ), ( std::vector<size_t>( { 0 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 22 ) ), ( std::vector<size_t>( { 0, 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 23 ) ), ( std::vector<size_t>( { 0, 1, 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 24 ) ), ( std::vector<size_t>( { 0, 1, 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 25 ) ), ( std::vector<size_t>( { 0, 1, 3 } ) ) );
        REQUIRE_EQUAL( ( findClose64( haystack, needle, 26 ) ), ( std::vector<size_t>( { 0, 1, 2, 3 } ) ) );
    }
}
#else
void
testCountDifferingBits8()
{}
#endif  // #ifdef __AVX2__


int main()
{
    testCountDifferingBits8();
    checkCountBits();
    checkHammingDistance();
    checkBKTree();
    return 0;
}
