
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


/**
 * Note that newer ones include older ones. Percentages in parentheses is the adoption rate according
 * to the Steam Survey June 2023 @see https://store.steampowered.com/hwsurvey/ -> Click on "Other Settings".
 * AVX was introduced with Intel Sandy Bridge in Q1 2011 and AMD Bulldozer in Q3 2011.
 * It makes sense to require users to have a processor that is not older than 12 years ...
 * Intrinsics are defined in
 * @see https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html
 * @see https://learn.microsoft.com/en-us/cpp/intrinsics/x86-intrinsics-list?view=msvc-170
 */
//#include <mmintrin.h>   // MMX
//#include <xmmintrin.h>  // SSE
//#include <emmintrin.h>  // SSE2
//#include <pmmintrin.h>  // SSE3   (100.00%)
//#include <tmmintrin.h>  // SSSE3  (100.00%)
//#include <smmintrin.h>  // SSE4.1  (99.43%)
//#include <nmmintrin.h>  // SSE4.2  (99.23%)
//#include <ammintrin.h>  // SSE4A   (31.95%) (AMD-only extension)
//#include <wmmintrin.h>  // AES
#include <immintrin.h>  // AVX (96.01%), AVX2 (88.88%), FMA
//#include <zmmintrin.h>  // AVX512 (9.71%)


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


#ifdef __AVX__
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


[[nodiscard]] std::vector<size_t>
findClose64Popcnt( const AlignedVector<uint64_t>& haystack,
                   const uint64_t                 needle,
                   const uint8_t                  distance )
{
    /* Broadcast value to SIMD vector entries for fast xor-ing. May generate vpbroadcastq.
     * @see https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html */
    const auto needleRepeated = _mm256_set1_epi64x( needle );

    std::vector<size_t> matches;

    static constexpr size_t SIMD_ELEMENTS = sizeof( __m256i ) / sizeof( haystack[0] );  // 32 B and 4 x 64-bit values!
    for ( size_t i = 0; i + SIMD_ELEMENTS - 1 < haystack.size(); i += SIMD_ELEMENTS ) {
        const auto values = _mm256_load_si256( reinterpret_cast<const __m256i*>( haystack.data() + i ) );
        const auto tested = values ^ needleRepeated;
    #if 0
        std::array<uint64_t, sizeof( __m256i ) / sizeof( uint64_t )> copy;
        std::memcpy( copy.data(), &tested, sizeof( tested ) );
        for ( size_t j = 0; j < copy.size(); ++j ) {
            if ( std::bitset<64>( copy[j] ).count() <= distance ) {
                matches.push_back( i + j );
            }
        }
    #else
        for ( size_t j = 0; j < SIMD_ELEMENTS; ++j ) {
            if ( std::bitset<64>( _mm256_extract_epi64( tested, SIMD_ELEMENTS - 1 - j ) ).count() <= distance ) {
                matches.push_back( i + j );
            }
        }
    #endif
    }

    return matches;
}


void
testCountDifferingBits8()
{
#if 0
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
#endif
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
#endif  // #ifdef __AVX__


void
benchmarkLinearHammingLookupOpenMP( const AlignedVector<uint64_t>& haystack,
                                    const uint64_t                 needle,
                                    const size_t                   distance )
{
    auto copy = haystack;
    AlignedVector<uint8_t> counts( haystack.size(), 0 );

    const auto t0 = now();

    auto* const copyData = copy.data();
    const auto* const haystackData = haystack.data();
    #pragma omp simd aligned(haystackData, copyData: 32)
    for ( size_t i = 0; i < haystack.size(); ++i ) {
        copyData[i] = haystackData[i] ^ needle;
    }
    #pragma omp simd aligned(copyData: 32)
    for ( size_t i = 0; i < haystack.size(); ++i ) {
        copyData[i] = std::bitset<64>( copyData[i] ).count() <= distance;
    }

    std::vector<size_t> positions;
    for ( size_t i = 0; i < copy.size(); ++i ) {
        if ( copyData[i] <= distance ) {
            positions.push_back( i );
        }
    }

    // 0.041 s for 10 M elements
    const auto t1 = now();

    std::cerr << "Found " << positions.size() << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " matching a distance " << distance << "\n";
    //for ( const auto& position : positions ) {
    //    const auto foundDistance = std::bitset<64>( copy[position] ^ needle ).count();
    //    std::cerr << "  " << position << " -> " << copy[position] << " distance: " << foundDistance << "\n";
    //}

    std::cerr << "hashes[332]: " << copy[331] << "\n";
    std::cerr << "hashes[332]: " << copy[332] << "\n";
    std::cerr << "hashes[333]: " << copy[333] << "\n";
    std::cerr << "hashes[333]: " << copy[334] << "\n";
}


#ifdef __AVX__
void
benchmarkLinearHammingLookupSIMD( const AlignedVector<uint64_t>& haystack,
                                  const uint64_t                 needle,
                                  const size_t                   distance )
{
    const auto t0 = now();
    const auto positions = findClose64( haystack, needle, distance );
    const auto t1 = now();

    std::cerr << "Found " << positions.size() << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " matching a distance " << distance << "\n";
    //for ( const auto& position : positions ) {
    //    const auto foundDistance = std::bitset<64>( haystack[position] ^ needle ).count();
    //    std::cerr << "  " << position << " -> " << haystack[position] << " distance: " << foundDistance << "\n";
    //}
}


void
benchmarkLinearHammingLookupSIMDPopcnt( const AlignedVector<uint64_t>& haystack,
                                        const uint64_t                 needle,
                                        const size_t                   distance )
{
    const auto t0 = now();
    const auto positions = findClose64Popcnt( haystack, needle, distance );
    const auto t1 = now();

    std::cerr << "Found " << positions.size() << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " matching a distance " << distance << "\n";
    //for ( const auto& position : positions ) {
    //    const auto foundDistance = std::bitset<64>( haystack[position] ^ needle ).count();
    //    std::cerr << "  " << position << " -> " << haystack[position] << " distance: " << foundDistance << "\n";
    //}
}
#endif


void
benchmarkLinearHammingLookupSimple( const AlignedVector<uint64_t>& haystack,
                                    const uint64_t                 needle,
                                    const size_t                   distance )
{
    const auto t0 = now();
    std::vector<size_t> positions;
    for ( size_t i = 0; i < haystack.size(); ++i ) {
        if ( std::bitset<64>( haystack[i] ^ needle ).count() <= distance ) {
            positions.push_back( i );
        }
    }
    const auto t1 = now();

    std::cerr << "Found " << positions.size() << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " matching a distance " << distance << "\n";
}



void
benchmarkLinearHammingLookupSimple2( const AlignedVector<uint64_t>& haystack,
                                     const uint64_t                 needle,
                                     const size_t                   distance )
{
    AlignedVector<uint64_t> xored( haystack.size() );
    const auto t0 = now();
    std::vector<size_t> positions;
    #pragma omp simd
    for ( size_t i = 0; i < haystack.size(); ++i ) {
        xored[i] = haystack[i] ^ needle;
    }
    for ( size_t i = 0; i < haystack.size(); ++i ) {
        if ( std::bitset<64>( xored[i] ).count() <= distance ) {
            positions.push_back( i );
        }
    }
    const auto t1 = now();

    std::cerr << "Found " << positions.size() << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " matching a distance " << distance << "\n";
}


void
benchmarkTreeHammingLookup64( const AlignedVector<uint64_t>& haystack,
                              const uint64_t                 needle,
                              const size_t                   distance,
                              const size_t                   maxElementCount )
{
    std::vector<uint64_t> copy( haystack.begin(), haystack.end() );
    const auto t0 = now();
    CppBKTree<uint64_t, size_t> bkTree( hammingDistance64, copy );
    bkTree.rebalance( maxElementCount );
    const auto t1 = now();
    std::cerr << "Creating BK tree from " << haystack.size() << " values took " << duration( t0, t1 ) << " s\n";

    const auto matches = bkTree.find( needle, distance );
    const auto t2 = now();
    std::cerr << "Found " << matches.size() << " matches in " << duration( t1, t2 ) << " s\n\n";

    const auto stats = bkTree.statistics();
    if ( false ) {
        std::cout << "nodeCount                : " << stats.nodeCount                << "\n";
        std::cout << "leafCount                : " << stats.leafCount                << "\n";
        std::cout << "valueCount               : " << stats.valueCount               << "\n";
        std::cout << "averageChildCountPerNode : " << stats.averageChildCountPerNode << "\n";
        std::cout << "maxDepth                 : " << stats.maxDepth                 << "\n";
        std::cout << "minChildrenPerNode       : " << stats.minChildrenPerNode       << "\n";
        std::cout << "maxChildrenPerNode       : " << stats.maxChildrenPerNode       << "\n";
        std::cout << "minPayloadsPerNode       : " << stats.minPayloadsPerNode       << "\n";
        std::cout << "maxPayloadsPerNode       : " << stats.maxPayloadsPerNode       << "\n";
        std::cout << "average payloads         : " << static_cast<double>( stats.valueCount ) /
                                                      static_cast<double>( stats.nodeCount ) << "\n";
        std::cout << "duplicateCount           : " << stats.duplicateCount           << "\n";
        std::cout << "valueBitCount            : " << stats.valueBitCount            << "\n";
    }
}


[[nodiscard]] std::vector<uint8_t>
toVector( uint64_t value )
{
    std::vector<uint8_t> result( sizeof( uint64_t ) );
    for ( size_t i = 0; i < sizeof( uint64_t ); ++i ) {
        result[i] = value & 0xFFULL;
        value >>= 8U;
    }
    return result;
}


void
benchmarkTreeHammingLookupVector( const AlignedVector<uint64_t>& haystack,
                                  const uint64_t               needle,
                                  const size_t                 distance,
                                  const size_t                 maxElementCount )
{
    std::vector<std::vector<uint8_t> > converted( haystack.size() );
    std::transform( haystack.begin(), haystack.end(), converted.begin(), toVector );

    const auto t0 = now();
    BKTree bkTree( hammingDistance, converted );
    bkTree.rebalance( maxElementCount );
    const auto t1 = now();
    std::cerr << "Creating std::vector<uint8_t> BK tree from " << haystack.size()
              << " values took " << duration( t0, t1 ) << " s\n";

    const auto matches = bkTree.find( toVector( needle ), distance );
    const auto t2 = now();
    std::cerr << "Found " << matches.size() << " matches in " << duration( t1, t2 ) << " s\n\n";
}


void
benchmarkHammingLookup( const size_t valueCount,
                        const size_t distance )
{
    std::cerr << "\n == Benchmarking for distance " << distance << " ==\n\n";

    const auto t0 = now();

    std::random_device randomDevice;
    std::default_random_engine randomEngine( 0 );
    std::uniform_int_distribution<uint64_t> distribution( 0 );

    AlignedVector<uint64_t> hashes( valueCount );
    if ( false ) {
        std::iota( hashes.begin(), hashes.end(), 0xFFFF'FFFF'FFFF'0000ULL );
    } else {
        for ( auto& h : hashes ) {
            h = distribution( randomEngine );
        }
    }

    const auto t1 = now();
    std::cerr << "Generating random numbers took " << duration( t0, t1 ) << " s\n";

    //const uint64_t needle( 0x1234'5678'90AB'CDEFULL );
    const auto needle = hashes[333];

#ifdef __AVX__
    // Found 31 positions out of 100000000 in 0.0885639 matching a distance 12
    std::cerr << "\n[benchmarkLinearHammingLookupSIMD]\n";
    benchmarkLinearHammingLookupSIMD( hashes, needle, distance );

    std::cerr << "\n[benchmarkLinearHammingLookupSIMDPopcnt]\n";
    benchmarkLinearHammingLookupSIMDPopcnt( hashes, needle, distance );
#endif

    // Found 31 positions out of 100000000 in 0.069386 matching a distance 12
    // ... Fuck... This simple version is actually slightly faster than the manual SIMD and much faster than OpenMP
    // "SIMD" extensions are not even used. Only -msse4.2 is important because it introduces popcnt, which
    // improves the time from 0.21 s down to 0.067 s, roughly 3-fold!
    std::cerr << "\n[benchmarkLinearHammingLookupSimple]\n";
    benchmarkLinearHammingLookupSimple( hashes, needle, distance );

    // Found 31 positions out of 100000000 in 0.154982 matching a distance 12
    //  -> This version with SIMD xoring and simple popcnt loop is twice as slow,
    //     meaning it probably is bandwidth bound. And this is with the std::vector
    //     allocation being disregarded, which would lead to a 0.36 s timing!
    //std::cerr << "\n[benchmarkLinearHammingLookupSimple2]\n";
    //benchmarkLinearHammingLookupSimple2( hashes, needle, distance );

    // Found 100000000 positions out of 100000000 in 0.973951 matching a distance 12
    //std::cerr << "\n[benchmarkLinearHammingLookupOpenMP]\n";
    //benchmarkLinearHammingLookupOpenMP( hashes, needle, distance );

    // Found 6718684 matches out of 10 M consecutive numbers in 0.983876 s for distance 12 (13.1 s tree creation)
    for ( const auto maxElementCount : { 16 } ) {
        std::cerr << "\n[benchmarkTreeHammingLookup64] rebalance( " << maxElementCount << " )\n";
        benchmarkTreeHammingLookup64( hashes, needle, distance, maxElementCount );
    }
    /**
     * Benchmark with 100 M elements:
     *
     * @verbatim
     * maxElementCount | Distance | Tree creation | Lookup      | Simple Lookup
     * ----------------+----------+---------------+------------ +--------------
     *         16      |     0    |  25.9439      | 4.226e-05   |
     *        256      |     0    |  16.8498      | 3.589e-05   |
     *       4096      |     0    |  11.4134      | 4.959e-05   |  0.0643504
     *      32768      |     0    |   9.05111     | 9.119e-05   |
     *     262144      |     0    |   7.07458     | 0.000205951 |
     * ----------------+----------+---------------+------------ +--------------
     *         16      |     2    |  23.5915      | 0.00124722  |
     *        256      |     2    |  16.8016      | 0.000644331 |
     *       4096      |     2    |  11.3062      | 0.000803542 |  0.0649045
     *      32768      |     2    |   9.08644     | 0.00193644  |
     *     262144      |     2    |   7.06035     | 0.00512268  |
     * ----------------+----------+---------------+------------ +--------------
     *         16      |    12    |  22.558       | 1.93299     |
     *        256      |    12    |  15.2452      | 0.468698    |
     *       4096      |    12    |  11.0815      | 0.208006    |  0.0654388
     *      32768      |    12    |   9.09776     | 0.190722    |
     *     262144      |    12    |   6.98312     | 0.190014    |
     * @endverbatim
     *
     * Observations:
     *
     *  - Tree creation becomes cheaper with larger chunk sizes because we don't have to split all that often.
     *  - Exact lookup is the fastest for the smallest tested chunk size of 256 but even for 32 K it is not that bad,
     *    i.e. it still is magnitudes faster than simple lookup.
     *  - maxElementCount == 1 could not be benchmarked because it takes too much memory!
     *  - Very inexact lookup with distance == 12 does not get faster anymore after 4096 chunk size.
     *
     * Conclusion:
     *
     *  - When only looking considering lookup times, 4 K would be the ideal chunk size.
     *  - Considering memory usage and tree creation times, a chunk size of 32 - 256 K is better
     *    while still being sufficiently faster for exact matching for the not shown maxElementCount == 1.
     */

    // Found 6718684 matches out of 10 M consecutive numbers in 1.0508 s for distance 12 (14.6 s tree creation)
    // Found 3 matches out of 10 M random numbers in 1.32147 s for distance 12 (14.6 s tree creation)
    //std::cerr << "\n[benchmarkTreeHammingLookupVector]\n";
    //benchmarkTreeHammingLookupVector( hashes, needle, distance );
}


void
benchmarkCppBkTreeHammingLookup()
{
    const auto t0 = now();

    std::random_device randomDevice;
    std::default_random_engine randomEngine( randomDevice() );
    std::uniform_int_distribution<uint64_t> distribution( 0 );

    AlignedVector<uint64_t> hashes( 100'000'000 );
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
    testCountDifferingBits8();
    //benchmarkHammingLookup( 100'000'000, 0 );
    //benchmarkHammingLookup( 100'000'000, 2 );
    //benchmarkHammingLookup( 100'000'000, 12 );

    checkCountBits();
    checkHammingDistance();
    checkBKTree();
    return 0;
}
