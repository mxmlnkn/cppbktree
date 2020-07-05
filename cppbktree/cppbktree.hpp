#pragma once

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <stack>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>


namespace
{
template<typename T,
         typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
size_t
countBits( T x )
{
    static_assert( std::numeric_limits<T>::radix == 2, "non-binary type" );

    constexpr int bitWidth = std::numeric_limits<T>::digits + std::numeric_limits<T>::is_signed;
    static_assert( bitWidth <= std::numeric_limits<unsigned long long>::digits,
                   "arg too wide for std::bitset() constructor");

    return std::bitset<bitWidth>( static_cast<typename std::make_unsigned<T>::type>( x ) ).count();
}


size_t
hammingDistance( const std::vector<uint8_t>& a,
                 const std::vector<uint8_t>& b )
{
    if ( a.size() != b.size() ) {
        throw std::invalid_argument( "Hashes to compare must be of equal length!" );
    }
    size_t nDifferingBits = 0;

    if ( a.size() % sizeof( uint64_t ) == 0 ) {
        const auto* const av = reinterpret_cast<const uint64_t*>( a.data() );
        const auto* const bv = reinterpret_cast<const uint64_t*>( b.data() );
        for ( size_t i = 0; i < a.size() / sizeof( uint64_t ); ++i ) {
            nDifferingBits += countBits( *av ^ *bv );
        }
    } else {
        for ( size_t i = 0; i < a.size(); ++i ) {
            nDifferingBits += countBits( static_cast<unsigned char>( a[i] ) ^
                                         static_cast<unsigned char>( b[i] ) );
        }
    }

    return nDifferingBits;
}


template<typename T>
std::ostream&
operator<<( std::ostream&         out,
            const std::vector<T>& values )
{
    std::ios ioState( nullptr );
    ioState.copyfmt( out );

    out << "{";
    for ( const auto& x : values ) {
        out << " ";

        //out << std::showbase << std::internal << std::setfill( '0' ) << std::hex << std::setw( 2 );
        out << static_cast<unsigned int>( x );
        out.copyfmt( ioState );
    }

    out << " }";
    return out;
}
}


template<typename T_ValueType,
         typename T_DistanceType>
class CppBKTree
{
public:
    using ValueType      = T_ValueType;
    using DistanceType   = T_DistanceType;
    using MetricFunction = std::function<DistanceType( const ValueType&, const ValueType& )>;

    struct Node
    {
        /**
         * Even if there are other values with a hamming distance of zero for this node,
         * it must suffice to only store one value because of the identity of indiscernibles axiom of metrics.
         */
        ValueType value;
        /**
         * for now only an ordinal number signifying when it was inserted.
         * 0 for the first inserted, 1 for the second inserted item, and so on.
         * If there are multiple values identical under the metric, then there might be more than one payload.
         */
        std::vector<size_t> payloads;
        std::map<DistanceType, std::unique_ptr<Node>> children;
    };

    struct TreeStatistics
    {
        size_t nodeCount = 0;
        size_t leafCount = 0;
        /** Might generally be larger than nodeCount as one node might have multiple identical values attached */
        size_t valueCount = 0;
        /**
         * Sum of all children ( nodeCount - 1 ( root ) ) divided
         * by the sum of all parents ( nodeCount - leafCount )
         */
        double averageChildCountPerNode = 0;
        /** Can be useful to determine whether the tree is evenly distributed */
        size_t maxDepth = 0;
        /** Leaf nodes are not counted because else this would always be 0 */
        size_t minChildrenPerNode = 0;
        size_t maxChildrenPerNode = 0;
        /** Can be derived as valueCount - nodeCount. */
        size_t duplicateCount = 0;
        /**
         * Basically the hash length and therefore gives the hamming distance range of [0,valueBitCount].
         * All values have the same bit count or else hammingDistance will throw.
         */
        size_t valueBitCount = 0;
    };

public:
    /**
     * Takes a vector of byte arrays to construct the tree from.
     */
    explicit
    CppBKTree( MetricFunction metricFunction,
               const std::vector<ValueType>& values = {} ) :
        m_metricFunction( metricFunction )
    {
        for ( const auto& value : values ) {
            add( value );
        }
    }

    explicit
    CppBKTree( const std::vector<ValueType>& values = {} ) :
        m_metricFunction( &hammingDistance )
    {
        for ( const auto& value : values ) {
            add( value );
        }
    }

    void
    add( const ValueType& value )
    {
        if ( !m_root ) {
            m_root = std::unique_ptr<Node>( new Node{ value, { m_itemCount++ }, {} } );
            return;
        }

        /* Descend into the tree along edges with equal distance to the node's item until there is no such edge. */
        auto* ppnode = &m_root;
        while ( true ) {
            const auto distance = m_metricFunction( (*ppnode)->value, value );

            /* Append identical value to this node */
            if ( distance == 0 ) {
                (*ppnode)->payloads.push_back( m_itemCount++ );
                break;
            }

            /* Insert a new child */
            auto& children = (*ppnode)->children;
            const auto child = children.find( distance );
            if ( child == children.end() ) {
                children.emplace( distance, std::unique_ptr<Node>( new Node{ value, { m_itemCount++ }, {} } ) );
                break;
            }

            /* Descend along the found edge */
            ppnode = &child->second;
        }
    }

    std::vector<size_t>
    find( const ValueType&             value,
          DistanceType                 distance,
          const std::unique_ptr<Node>* root = nullptr ) const
    {
        if ( root == nullptr ) {
            if ( !m_root ) {
                return {};
            }
            return find( value, distance, &m_root );
        }

        std::vector<size_t> result;

        std::stack<const std::unique_ptr<Node>*> candidates;
        candidates.push( root );
        while ( !candidates.empty() ) {
            const auto ppnode = candidates.top();
            candidates.pop();
            const auto distanceToThisNode = m_metricFunction( (*ppnode)->value, value );

            if ( distanceToThisNode <= distance ) {
                const auto& payloads = (*ppnode)->payloads;
                std::copy( payloads.begin(), payloads.end(), std::back_inserter( result ) );
            }

            /* The map of children is inherently sorted by distance, so it suffices
             * to search for a lower bound and then iterate to the upper bound.
             * Be pedantic for overflow checks! */
            const auto lowerBound = distanceToThisNode >= distance ? distanceToThisNode - distance : 0;
            const auto upperBound = std::numeric_limits<DistanceType>::max() - distanceToThisNode >= distance
                                    ? distanceToThisNode + distance
                                    : std::numeric_limits<DistanceType>::max();
            for ( auto candidate = (*ppnode)->children.lower_bound( lowerBound );
                  ( candidate->first <= upperBound ) && ( candidate != (*ppnode)->children.end() );
                  ++candidate )
            {
                /**
                 * Consider a hash with n bits. The distance will range in [0,n]. The hash will have at most 2^n
                 * distinct values and therefore tree nodes. Assuming a balanced tree, a depth d will be able to
                 * encode n^d values. As there are only 2^n values possible, the max depth D is given as 2^n = n^D.
                 * This is equivalent to n log(2) = D log(n) giving D = n log(2) / log(n).
                 * For most hashes, recursion shouldn't be a problem but I don't know. I arbitrarily decided
                 * to avoid recursion using a stack. This might also help avoiding multiple copies.
                 * In the worst case, the stack will have to store d * n pointers because the push pop mechanism
                 * basically boils down to a depth-first search.
                 * Note that the maxium depth for a very uneven tree would be D = n if we inserted successively
                 * elements with distance one from each other. This would only require n elements so deep trees
                 * can do happen for few elements.
                 * I compared three versions:
                 *  1. A recursive version, which is still in the code with FIND_RECURSIVE = true
                 *  2. A depth-first search using a stack, which is still in the code with FIND_RECURSIVE = false
                 *  3. A temporary version using a vector with a capacity of maxDistance * log(2) / log( maxDistance ).
                 *     It might have been better to simply use .reserve( maxDistance ) but I checked with debug output
                 *     and the vector size never seemed to exceed the capacity, so there should be no reallocations.
                 * The benchmark also always compares the results to the pybktree implementation, so the results should
                 * be correct.
                 * Conclusion: All versions suffer a performance degradation after 2e4 (1), 2e4 (2), 3e3 (3) elements.
                 *             The recursive version is pretty much identical to the version using the stack.
                 *             The vector version has better performance than the other versions for 3e3 elements,
                 *             but then the runtime suddenly spikes to 1e-4 seconds and the scalings for all different
                 *             distance threshold begin to overlap, probably because some huge offset make them
                 *             pretty much indiscernible.
                 *             For distances <= 8 (n=64), this version is actually slower than pybktree!
                 */
                if ( FIND_RECURSIVE ) {
                    const auto recursiveResult = find( value, distance, &candidate->second );
                    result.insert( result.end(), recursiveResult.begin(), recursiveResult.end() );
                } else {
                    candidates.push( &candidate->second );
                }
            }
        }

        return result;
    }

    size_t
    size() const
    {
        return m_itemCount;
    }

    /**
     * Returns some statistics about the tree
     */
    TreeStatistics
    statistics() const
    {
        if ( !m_root ) {
            return TreeStatistics();
        }

        TreeStatistics result;
        result.valueBitCount = m_root->value.size() * std::numeric_limits<uint8_t>::digits;
        result.minChildrenPerNode = std::numeric_limits<decltype( result.minChildrenPerNode )>::max();

        std::stack<std::pair<const std::unique_ptr<Node>*, /* depth */ size_t> > nodesToProcess;
        nodesToProcess.push( std::make_pair( &m_root, 1 ) );
        while ( !nodesToProcess.empty() ) {
            const auto nodeToProcess = nodesToProcess.top();
            const auto ppnode = nodeToProcess.first;
            const auto nodeDepth = nodeToProcess.second;
            nodesToProcess.pop();

            const auto& children = (*ppnode)->children;

            result.nodeCount += 1;
            result.valueCount += (*ppnode)->payloads.size();
            result.maxDepth = std::max( result.maxDepth, nodeDepth );
            if ( children.empty() ) {
                result.leafCount += 1;
            } else {
                result.minChildrenPerNode = std::min( result.minChildrenPerNode, children.size() );
                result.maxChildrenPerNode = std::max( result.maxChildrenPerNode, children.size() );
            }

            for ( auto child = children.begin(); child != children.end(); ++child ) {
                nodesToProcess.push( std::make_pair( &child->second, nodeDepth + 1 ) );
            }
        }

        result.duplicateCount = result.valueCount - result.nodeCount;
        result.averageChildCountPerNode = static_cast<double>( result.nodeCount - 1 )
                                          / ( result.nodeCount - result.leafCount );

        return result;
    }

private:
    std::unique_ptr<Node> m_root;
    size_t m_itemCount = 0;
    const MetricFunction m_metricFunction;
    static constexpr bool FIND_RECURSIVE = false;
};
