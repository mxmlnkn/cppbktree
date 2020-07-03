#pragma once

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <stack>
#include <stdexcept>
#include <type_traits>
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
}


class BKTree
{
public:
    using Distance = size_t;

    struct Node
    {
        /**
         * Even if there are other values with a hamming distance of zero for this node,
         * it must suffice to only store one value because of the identity of indiscernibles axiom of metrics.
         */
        std::vector<uint8_t> value;
        /**
         * for now only an ordinal number signifying when it was inserted.
         * 0 for the first inserted, 1 for the second inserted item, and so on.
         * If there are multiple values identical under the metric, then there might be more than one payload.
         */
        std::vector<size_t> payloads;
        std::map<Distance, std::unique_ptr<Node>> children;
    };

public:
    /**
     * Takes a vector of byte arrays to construct the tree from.
     */
    explicit
    BKTree( const std::vector<std::vector<uint8_t>>& values = {} )
    {
        for ( const auto& value : values ) {
            add( value );
        }
    }

    void
    add( const std::vector<uint8_t>& value )
    {
        if ( !m_root ) {
            m_root = std::unique_ptr<Node>( new Node{ value, { m_itemCount++ }, {} } );
            return;
        }

        /* Descend into the tree along edges with equal distance to the node's item until there is no such edge. */
        auto* ppnode = &m_root;
        while ( true ) {
            const auto distance = m_distance( (*ppnode)->value, value );

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
    find( const std::vector<uint8_t>& value,
          Distance                    distance ) const
    {
        if ( !m_root ) {
            return {};
        }

        std::vector<size_t> result;

        std::stack<const std::unique_ptr<Node>*> candidates;
        candidates.push( &m_root );
        while ( !candidates.empty() ) {
            const auto ppnode = candidates.top();
            candidates.pop();
            const auto distanceToThisNode = m_distance( (*ppnode)->value, value );

            if ( distanceToThisNode <= distance ) {
                const auto& payloads = (*ppnode)->payloads;
                std::copy( payloads.begin(), payloads.end(), std::back_inserter( result ) );
            }

            /* The map of children is inherently sorted by distance, so it suffices
             * to search for a lower bound and then iterate to the upper bound.
             * Be pedantic for overflow checks! */
            const auto lowerBound = distanceToThisNode >= distance ? distanceToThisNode - distance : 0;
            const auto upperBound = std::numeric_limits<Distance>::max() - distanceToThisNode >= distance
                                    ? distanceToThisNode + distance
                                    : std::numeric_limits<Distance>::max();
            for ( auto candidate = (*ppnode)->children.lower_bound( lowerBound );
                  ( candidate->first <= upperBound ) && ( candidate != (*ppnode)->children.end() );
                  ++candidate )
            {
                /* Consider a hash with n bits. The distance will range in [0,n]. The hash will have at most 2^n
                 * distinct values and therefore tree nodes. Assuming a balanced tree, a depth d will be able to
                 * encode n^d values. As there are only 2^n values possible, the max depth D is given as 2^n = n^D.
                 * This is equivalent to n log(2) = D log(n) giving D = n log(2) / log(n).
                 * For most hashes, recursion shouldn't be a problem but I don't know. I arbitrarily decided
                 * to avoid recursion using a stack. This might also help avoiding multiple copies. */
                candidates.push( &candidate->second );
            }
        }

        return result;
    }

private:
    std::unique_ptr<Node> m_root;
    size_t m_itemCount = 0;
    std::function<Distance( const std::vector<uint8_t>&, const std::vector<uint8_t>& )> m_distance = &hammingDistance;
};
