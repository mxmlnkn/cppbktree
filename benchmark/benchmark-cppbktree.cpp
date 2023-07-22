
#include <AlignedAllocator.hpp>
#include <TestHelpers.hpp>

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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
#endif


#ifdef _OPENMP
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
#endif


#ifdef __AVX2__
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


#ifdef _OPENMP
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
#endif


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

#ifdef __AVX2__
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

#ifdef _OPENMP
    // Found 31 positions out of 100000000 in 0.154982 matching a distance 12
    //  -> This version with SIMD xoring and simple popcnt loop is twice as slow,
    //     meaning it probably is bandwidth bound. And this is with the std::vector
    //     allocation being disregarded, which would lead to a 0.36 s timing!
    std::cerr << "\n[benchmarkLinearHammingLookupSimple2]\n";
    benchmarkLinearHammingLookupSimple2( hashes, needle, distance );

    // Found 100000000 positions out of 100000000 in 0.973951 matching a distance 12
    std::cerr << "\n[benchmarkLinearHammingLookupOpenMP]\n";
    benchmarkLinearHammingLookupOpenMP( hashes, needle, distance );
#endif

    // Found 6718684 matches out of 10 M consecutive numbers in 0.983876 s for distance 12 (13.1 s tree creation)
    //for ( const auto maxElementCount : { 8ULL * 1024ULL } ) {
    //    std::cerr << "\n[benchmarkTreeHammingLookup64] rebalance( " << maxElementCount << " )\n";
    //    benchmarkTreeHammingLookup64( hashes, needle, distance, maxElementCount );
    //}
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
benchmarkLinearHammingLookupSimpleBatch( const AlignedVector<uint64_t>& haystack,
                                         const std::vector<uint64_t>&   needles,
                                         const size_t                   distance )
{
    const auto t0 = now();
    std::vector<std::vector<size_t> > positions( needles.size() );
    for ( size_t i = 0; i < haystack.size(); ++i ) {
        for ( size_t j = 0; j < needles.size(); ++j ) {
            if ( std::bitset<64>( haystack[i] ^ needles[j] ).count() <= distance ) {
                positions[j].push_back( i );
            }
        }
    }
    const auto t1 = now();

    const auto totalMatches = std::accumulate( positions.begin(), positions.end(), uint64_t( 0 ),
                                               [] ( const uint64_t sum, const auto& v ) { return sum + v.size(); } );
    std::cerr << "Found " << totalMatches << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " for " << needles.size()
              << " needles matching a distance " << distance << "\n";
}


void
benchmarkLinearHammingLookupSimpleBatch2( const AlignedVector<uint64_t>& haystack,
                                          const std::vector<uint64_t>&   needles,
                                          const size_t                   distance )
{
    const auto t0 = now();
    std::vector<std::vector<size_t> > positions( needles.size() );
    static constexpr size_t CHUNK_SIZE = 1024;
    for ( size_t k = 0; k < haystack.size(); k += CHUNK_SIZE ) {
        const auto maximum = std::min( haystack.size(), k + CHUNK_SIZE );
        for ( size_t j = 0; j < needles.size(); ++j ) {
            for ( size_t i = k; i < maximum; ++i ) {
                if ( std::bitset<64>( haystack[i] ^ needles[j] ).count() <= distance ) {
                    positions[j].push_back( i );
                }
            }
        }
    }
    const auto t1 = now();

    const auto totalMatches = std::accumulate( positions.begin(), positions.end(), uint64_t( 0 ),
                                               [] ( const uint64_t sum, const auto& v ) { return sum + v.size(); } );
    std::cerr << "Found " << totalMatches << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " for " << needles.size()
              << " needles matching a distance " << distance << "\n";
}


#ifdef __AVX2__
void
benchmarkLinearHammingLookupSIMDBatch( const AlignedVector<uint64_t>& haystack,
                                       const std::vector<uint64_t>&   needles,
                                       const size_t                   distance )
{
    const auto t0 = now();

    std::vector<std::vector<size_t> > positions( needles.size() );

    /* Add 1 to distance because there is only == or > for 16-bit integer AVX2 instructinos. */
    const auto distanceRepeated = _mm256_set1_epi64x( distance == std::numeric_limits<uint8_t>::max()
                                                      ? distance : distance + 1 );

    /* Duplicate 128-bit LUT to 256-bit so that it works with the peculiar 256-bit shuffle version. */
    const auto bitCountLUT128 =
        _mm_load_si128( reinterpret_cast<const __m128i*>( BIT_COUNT_LUT.data() ) );  // 128 b = 16 B LUT size
    const auto bitCountLUT = _mm256_set_m128i( bitCountLUT128, bitCountLUT128 );

    const auto LOW_NIBBLES_SET = _mm256_set1_epi8( 0x0F );
    const auto LOW_BYTES_SET = _mm256_set1_epi16( 0xFF );

    static constexpr size_t SIMD_ELEMENTS = sizeof( __m256i ) / sizeof( haystack[0] );  // 32 B and 4 x 64-bit values!
    for ( size_t i = 0; i + SIMD_ELEMENTS - 1 < haystack.size(); i += SIMD_ELEMENTS ) {
        const auto values = _mm256_load_si256( reinterpret_cast<const __m256i*>( haystack.data() + i ) );

        for ( size_t iNeedle = 0; iNeedle < needles.size(); ++iNeedle ) {
            const auto needleRepeated = _mm256_set1_epi64x( needles[iNeedle] );

            const auto tested = values ^ needleRepeated;
            const auto bitCounts8 = _mm256_shuffle_epi8( bitCountLUT, tested & LOW_NIBBLES_SET ) +
                                    _mm256_shuffle_epi8( bitCountLUT, ( tested >> 4U ) & LOW_NIBBLES_SET );

            const auto bitCounts16 = ( bitCounts8  & LOW_BYTES_SET ) + _mm256_srli_epi64( bitCounts8 ,  8U );
            const auto bitCounts32 = ( bitCounts16 & LOW_BYTES_SET ) + _mm256_srli_epi64( bitCounts16, 16U );
            const auto bitCounts64 = ( bitCounts32 & LOW_BYTES_SET ) + _mm256_srli_epi64( bitCounts32, 32U );

            /* Move bytes with counts for consecutive bits to the least significant positions and cast down. */
            auto result = static_cast<uint32_t>(
                _mm256_movemask_epi8( _mm256_cmpgt_epi16( distanceRepeated, bitCounts64 ) ) );
            for ( size_t k = 0; result != 0; result >>= sizeof( needles[0] ), ++k ) {
                if ( ( result & 1U ) > 0 ) {
                    positions[iNeedle].push_back( i + k );
                }
            }
        }
    }

    const auto t1 = now();
    const auto totalMatches = std::accumulate( positions.begin(), positions.end(), uint64_t( 0 ),
                                               [] ( const uint64_t sum, const auto& v ) { return sum + v.size(); } );
    std::cerr << "Found " << totalMatches << " positions out of " << haystack.size()
              << " in " << duration( t0, t1 ) << " matching a distance " << distance << "\n";
}
#endif  // ifdef __AVX2__


void
benchmarkBatchHammingLookup( const size_t valueCount,
                             const size_t distance )
{
    std::cerr << "\n == Benchmarking for distance " << distance << " ==\n";

    const auto t0 = now();

    std::random_device randomDevice;
    std::default_random_engine randomEngine( 0 );
    std::uniform_int_distribution<uint64_t> distribution( 0 );

    AlignedVector<uint64_t> hashes( valueCount );
    for ( auto& h : hashes ) {
        h = distribution( randomEngine );
    }

    const auto t1 = now();
    std::cerr << "Generating random numbers took " << duration( t0, t1 ) << " s\n";

    const std::vector<uint64_t> needles = {
        hashes[333], hashes[334], hashes[335], hashes[336],
        hashes[1333], hashes[1334], hashes[1335], hashes[1336],
        hashes[21333], hashes[21334], hashes[21335], hashes[21336],
        hashes[421333], hashes[421334], hashes[421335], hashes[421336],
    };

    std::cerr << "[benchmarkLinearHammingLookupSimpleBatch] ";
    benchmarkLinearHammingLookupSimpleBatch( hashes, needles, distance );
    std::cerr << "[benchmarkLinearHammingLookupSimpleBatch2] ";
    benchmarkLinearHammingLookupSimpleBatch2( hashes, needles, distance );

    std::cerr << "[benchmarkLinearHammingLookupSIMDBatch] ";
    benchmarkLinearHammingLookupSIMDBatch( hashes, needles, distance );
}


int main()
{
    benchmarkBatchHammingLookup( 100'000'000, 0 );
    benchmarkBatchHammingLookup( 100'000'000, 2 );
    benchmarkBatchHammingLookup( 100'000'000, 12 );

    benchmarkHammingLookup( 100'000'000, 0 );
    benchmarkHammingLookup( 100'000'000, 2 );
    benchmarkHammingLookup( 100'000'000, 12 );
    return 0;
}
