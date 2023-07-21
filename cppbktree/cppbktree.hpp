#pragma once

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <stack>
#include <stdexcept>
#include <tuple>
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


[[maybe_unused]] size_t
hammingDistance64( const uint64_t a,
                   const uint64_t b )
{
    return countBits( a ^ b );
}


template<typename T>
void
serializeVector( std::ostream&         out,
                 const std::vector<T>& value )
{
    const uint64_t size = value.size();
    out.write( reinterpret_cast<const char*>( &size ), sizeof( size ) );
    out.write( reinterpret_cast<const char*>( value.data() ), value.size() * sizeof( value[0] ) );
}


/**
 * @return true if reading was successful
 */
template<typename T>
bool
deserializeVector( std::istream&   in,
                   std::vector<T>* value )
{
    if ( value == nullptr ) {
        throw std::invalid_argument( "Result location may not be nullptr!" );
    }

    uint64_t size = 0;
    in.read( reinterpret_cast<char*>( &size ), sizeof( size ) );
    if ( !in.good() ) {
        return false;
    }

    value->resize( size );
    in.read( reinterpret_cast<char*>( value->data() ), value->size() * sizeof( value->front() ) );
    return in.good();
}


template<typename T,
         typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
void
serialize( std::ostream& out,
           T             value )
{
    out.write( reinterpret_cast<const char*>( &value ), sizeof( value ) );
}


template<typename T,
         typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
bool
deserialize( std::istream& in,
             T*            value )
{
    if ( value == nullptr ) {
        throw std::invalid_argument( "Result location may not be nullptr!" );
    }
    in.read( reinterpret_cast<char*>( value ), sizeof( *value ) );
    return in.good();
}


static const std::string DATA_MAGIC_BYTES = "CPPBKTREE";
static constexpr uint8_t DATA_VERSION = 0;
}


template<typename T_ValueType,
         typename T_DistanceType>
class CppBKTree
{
public:
    using ValueType      = T_ValueType;
    using DistanceType   = T_DistanceType;
    using MetricFunction = std::function<DistanceType( const ValueType&, const ValueType& )>;

    /* Benchmarks show that linear lookup of 1 to ~1000 all take roughly the same time.
     * Only after that, the cost begins to scale linearly up. */
    static constexpr size_t CHUNK_SIZE = 1024;

    struct Node
    {
        /**
         * This is the bisection value for this node. For leaf nodes, it is only tentative and is effectively unused.
         */
        ValueType referenceValue;

        /**
         * Even if the CHUNK_SIZE is 1, there might be other values with a hamming distance of zero for this node.
         */
        std::vector<ValueType> values;
        /**
         * for now only an ordinal number signifying when it was inserted.
         * 0 for the first inserted, 1 for the second inserted item, and so on.
         * If there are multiple values identical under the metric, then there might be more than one payload.
         */
        std::vector<size_t> payloads;
        std::map<DistanceType, std::unique_ptr<Node> > children;

    public:
        void
        addToChild( ValueType             value,
                    size_t                payload,
                    const MetricFunction& metricFunction )
        {
            const auto distance = metricFunction( referenceValue, value );
            const auto child = children.find( distance );
            if ( child == children.end() ) {
                auto node = std::unique_ptr<Node>( new Node{ value, { value }, { payload }, {} } );
                children.emplace( distance, std::move( node ) );
            } else {
                child->second->add( std::move( value ), payload, metricFunction );
            }
        }

        void
        add( ValueType             value,
             size_t                payload,
             const MetricFunction& metricFunction )
        {
            /** Simply append to leaf nodes. They are split inside @ref rebalance. */
            if ( children.empty() && ( values.size() == payloads.size() ) ) {
                values.emplace_back( std::move( value ) );
                payloads.emplace_back( payload );
            } else {
                addToChild( std::move( value ), payload, metricFunction );
            }
        }

        void
        rebalance( const MetricFunction& metricFunction )
        {
            /* Would be good to have a maximum possible hamming distance safely derived
             * from ValueType and MetricFunction. But even if we had that, the ideal chunk size also depends
             * on the benchmarks, which are dependant on both. Not possibly to do it generically autlomatically.
             * Simply assume std::vecto<uint8_t> with 8 values (effectively uint64_t) for now.
             * One problem here is that a single node split on average would lead to maximum distance more nodes.
             * For uint64_t, this means almost two magnitudes more nodes for a single split.
             * This makes the tradeoff consideration for splitting vs not splitting difficult.
             * For large distance lookups (distance > maximum distance / 4), splitting is not necessary at all.
             * This means, we are doing this to optimize small distances such as distance = 0.
             * For distance = 0, we need to find splitting makes almost always sense, except if a linear lookup
             * is not reduced any further, which is the case for element counts < 1k.
             * However, splitting for more than 1k would result in sub nodes with each only ~16 elements!
             * This would impede lookup for large distances!
             * Therefore, do a compromise and split at 10k. Note that 100k is the point at which linear lookup
             * becomes slower than BK tree lookup for distance == 0. So any split point < 100k would be fine
             * even if not ideal. */
            if ( payloads.size() > 32ULL * 1024ULL ) {
                referenceValue = values.back();
                std::vector<size_t> zeroDistancePayloads;
                for ( size_t i = 0; i < std::min( values.size(), payloads.size() ); ++i ) {
                    const auto distance = metricFunction( referenceValue, values[i] );
                    if ( distance == 0 ) {
                        zeroDistancePayloads.emplace_back( payloads[i] );
                    } else {
                        addToChild( std::move( values[i] ), payloads[i], metricFunction );
                    }
                }
                values.clear();
                payloads = std::move( zeroDistancePayloads );
            }

            for ( const auto& [_, child] : children ) {
                child->rebalance( metricFunction );
            }
        }

        [[nodiscard]] std::vector<size_t>
        find( const ValueType&      value,
              DistanceType          distance,
              const MetricFunction& metricFunction ) const
        {
            std::vector<size_t> result;

            if ( children.empty() && ( values.size() == payloads.size() ) ) {
                for ( size_t i = 0; i < values.size(); ++i ) {
                    if ( metricFunction( value, values[i] ) <= distance ) {
                        result.emplace_back( payloads[i] );
                    }
                }
                return result;
            }

            /* If there are children or more payloads than values, then @ref referenceValue should be valid! */
            const auto distanceToThisNode = metricFunction( value, referenceValue );
            if ( distanceToThisNode <= distance ) {
                std::copy( payloads.begin(), payloads.end(), std::back_inserter( result ) );
            }

            /* The map of children is inherently sorted by distance, so it suffices
             * to search for a lower bound and then iterate to the upper bound.
             * Be pedantic for overflow checks! */
            const auto lowerBound = distanceToThisNode >= distance ? distanceToThisNode - distance : 0;
            const auto upperBound = std::numeric_limits<DistanceType>::max() - distanceToThisNode >= distance
                                    ? distanceToThisNode + distance
                                    : std::numeric_limits<DistanceType>::max();
            for ( auto candidate = children.lower_bound( lowerBound );
                  ( candidate->first <= upperBound ) && ( candidate != children.end() );
                  ++candidate )
            {
                const auto recursiveResult = candidate->second->find( value, distance, metricFunction );
                result.insert( result.end(), recursiveResult.begin(), recursiveResult.end() );
            }

            return result;
        }

        void
        serialize( std::ostream& out,
                   uint32_t      depth ) const
        {
            serializeVector( out, referenceValue );
            serializeVector( out, values );
            serializeVector( out, payloads );

            for ( const auto& child : children ) {
                ::serialize( out, depth );
                ::serialize( out, child.first );
                child.second->serialize( out, depth + 1 );
            }
        }

        /**
         * @return number of successfully deserialized entries.
         */
        size_t
        deserialize( std::istream& in,
                     uint32_t      depth )
        {
            static_assert( std::is_same<ValueType, std::vector<uint8_t> >::value,
                           "Not yet working for generic value types!" );
            deserializeVector( in, &referenceValue );
            deserializeVector( in, &values );
            deserializeVector( in, &payloads );

            /* File is not good if EOF is reached. EOF is only reached when trying to read bytes after it.
             * Reading the very last byte in the file successfully, will not set the EOF flag yet!
             * So, if it is set, then deserializing one of the above contents already failed. */
            if ( !in.good() ) {
                return 0;
            }
            size_t deserializedPayloadsCount = payloads.size();

            while ( in.good() ) {
                /* Peek at the depth of the next entry. Return if it isn't for us. */
                const auto oldPos = in.tellg();
                uint32_t entryDepth = 0;
                ::deserialize( in, &entryDepth );

                if ( !in.good() ) {
                    /* Simply no more entries to read but the current node was deserialized successfully. */
                    return deserializedPayloadsCount;
                }

                if ( entryDepth < depth ) {
                    /* Rewind, so that the responsible parent can reread and check the depth. */
                    in.seekg( oldPos );
                    return deserializedPayloadsCount;
                }

                if ( entryDepth > depth + 1 ) {
                    throw std::invalid_argument( "Data file seems to have skipped some tree levels when descending!" );
                }

                /* Entry belongs to us, so deserialize it */
                DistanceType distance;
                ::deserialize( in, &distance );
                if ( !in.good() ) {
                    throw std::invalid_argument( "Valid depth found but not followed by distance value. "
                                                 "Incomplete data file?" );
                }

                auto node = std::unique_ptr<Node>( new Node() );
                const auto deserializedChildrenCount = node->deserialize( in, depth + 1 );
                if ( deserializedChildrenCount > 0 ) {
                    children.emplace( distance, std::move( node ) );
                    deserializedPayloadsCount += deserializedChildrenCount;
                }
            }

            return deserializedPayloadsCount;
        }

        bool
        operator==( const Node& other ) const
        {
            if ( ( referenceValue  != other.referenceValue  ) ||
                 ( values          != other.values          ) ||
                 ( payloads        != other.payloads        ) ||
                 ( children.size() != other.children.size() ) )
            {
                return false;
            }

            using ChildKeyValue = decltype( *children.begin() );

            const auto childrenAreEqual =
                [] ( const ChildKeyValue& child1, const ChildKeyValue& child2 )
                {
                    if ( ( child1.first != child2.first ) ||
                         ( static_cast<bool>( child1.second ) != static_cast<bool>( child2.second ) ) )
                    {
                        return false;
                    }

                    if ( child1.second && child2.second ) {
                        return *child1.second == *child2.second;
                    }

                    return true;
                };

            return std::equal( children.begin(), children.end(), other.children.begin(), childrenAreEqual );
        }


        void
        print( std::ostream&    out,
               uint32_t         depth = 0 ) const
        {
            std::ios ioState( nullptr );
            ioState.copyfmt( out );

            out << "Value: " << referenceValue << ", Values: " << values << ", Payloads: " << payloads << std::endl;

            for ( const auto& child : children ) {
                out << std::setw( ( depth + 1 ) * 2 ) << std::setfill( ' ' ) << ""
                    << std::setw( 8 ) << child.first;
                out.copyfmt( ioState );
                out << " : ";
                child.second->print( out, depth + 1 );
            }
        }
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

        size_t minPayloadsPerNode{ 0 };
        size_t maxPayloadsPerNode{ 0 };
    };

public:
    /**
     * Takes a vector of byte arrays to construct the tree from.
     */
    explicit
    CppBKTree( MetricFunction metricFunction,
               std::vector<ValueType> values = {} ) :
        m_itemCount( values.size() ),
        m_metricFunction( metricFunction )
    {
        if ( !values.empty() ) {
            std::vector<size_t> payloads( m_itemCount );
            std::iota( payloads.begin(), payloads.end(), 0 );
            m_root = std::unique_ptr<Node>( new Node{ {}, std::move( values ), std::move( payloads ), {} } );
        }
        rebalance();
    }

    explicit
    CppBKTree( std::vector<ValueType> values = {} ) :
        CppBKTree( &hammingDistance, std::move( values ) )
    {}

    explicit
    CppBKTree( const std::string& fileName ) :
        m_metricFunction( &hammingDistance )
    {
        std::ifstream inputFile( fileName );
        deserialize( inputFile );
    }

    void
    add( ValueType value )
    {
        if ( m_root ) {
            m_root->add( std::move( value ), m_itemCount++, m_metricFunction );
        } else {
            m_root = std::unique_ptr<Node>( new Node{ {}, { value }, { m_itemCount++ }, /* children */ {} } );
        }
    }

    void
    rebalance()
    {
        if ( m_root ) {
            m_root->rebalance( m_metricFunction );
        }
    }

    [[nodiscard]] std::vector<size_t>
    find( const ValueType& value,
          DistanceType     distance ) const
    {
        if ( m_root ) {
            return m_root->find( value, distance, m_metricFunction );
        }
        return {};
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

        if constexpr ( std::is_integral_v<ValueType> ) {
            result.valueBitCount = std::numeric_limits<ValueType>::digits;
        } else {
            const auto elementCount = m_root->values.empty()
                                      ? m_root->referenceValue.size()
                                      : m_root->values.front().size();
            result.valueBitCount = elementCount * std::numeric_limits<uint8_t>::digits;
        }

        result.minChildrenPerNode = std::numeric_limits<decltype( result.minChildrenPerNode )>::max();
        result.maxChildrenPerNode = std::numeric_limits<decltype( result.minChildrenPerNode )>::min();

        result.minPayloadsPerNode = std::numeric_limits<decltype( result.minPayloadsPerNode )>::max();
        result.maxPayloadsPerNode = std::numeric_limits<decltype( result.minPayloadsPerNode )>::min();

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

            const auto& payloads = (*ppnode)->payloads;
            result.minPayloadsPerNode = std::min( result.minPayloadsPerNode, payloads.size() );
            result.maxPayloadsPerNode = std::max( result.maxPayloadsPerNode, payloads.size() );

            for ( auto child = children.begin(); child != children.end(); ++child ) {
                nodesToProcess.push( std::make_pair( &child->second, nodeDepth + 1 ) );
            }
        }

        if ( result.minChildrenPerNode > result.maxChildrenPerNode ) {
            result.minChildrenPerNode = 0;
            result.maxChildrenPerNode = 0;
        }

        result.duplicateCount = result.valueCount - result.nodeCount;
        result.averageChildCountPerNode = static_cast<double>( result.nodeCount - 1 )
                                          / ( result.nodeCount - result.leafCount );

        return result;
    }

    void
    serialize( const std::string& fileName ) const
    {
        std::ofstream outputFile( fileName );
        serialize( outputFile );
    }

    void
    serialize( std::ostream& out ) const
    {
        out.write( reinterpret_cast<const char*>( DATA_MAGIC_BYTES.data() ),
                   DATA_MAGIC_BYTES.size() * sizeof( DATA_MAGIC_BYTES.front() ) );
        out.write( reinterpret_cast<const char*>( &DATA_VERSION ), sizeof( DATA_VERSION ) );

        if ( m_root ) {
            m_root->serialize( out, 0 );
        }
    }

    void
    deserialize( std::istream& in )
    {
        std::string buffer( DATA_MAGIC_BYTES.size(), '\0' );
        in.read( reinterpret_cast<char*>( &buffer[0] ), buffer.size() * sizeof( buffer.front() ) );
        if ( ( buffer != DATA_MAGIC_BYTES ) || !in.good() ) {
            throw std::invalid_argument( "Could not verify magic bytes for given file!" );
        }

        uint8_t version = DATA_VERSION + 1; // use a different initial value from what it should be to detect errors!
        in.read( reinterpret_cast<char*>( &version ), sizeof( version ) );
        if ( version != DATA_VERSION ) {
            throw std::invalid_argument( "Mismatching data file version for given file!" );
        }

        if ( in.good() ) {
            m_root = std::unique_ptr<Node>( new Node() );
            m_itemCount = m_root->deserialize( in, 0 );
            if ( m_itemCount == 0 ) {
                m_root.release();
            }
        }
    }

    bool
    operator==( const CppBKTree& other ) const
    {
        if ( ( static_cast<bool>( m_root ) != static_cast<bool>( other.m_root ) ) ||
             ( m_itemCount != other.m_itemCount ) )
        {
             return false;
        }

        if ( m_root && other.m_root ) {
            return *m_root == *other.m_root;
        }

        return true;

    }

    friend std::ostream&
    operator<<( std::ostream&    out,
                const CppBKTree& tree )
    {
        out << "Tree with " << tree.m_itemCount << " items:\n";
        if ( tree.m_root ) {
            tree.m_root->print( out, 1 );
        } else {
            out << "<empty tree>";
        }
        out << std::endl;
        return out;
    }

private:
    std::unique_ptr<Node> m_root;
    size_t m_itemCount = 0;
    const MetricFunction m_metricFunction;
};
