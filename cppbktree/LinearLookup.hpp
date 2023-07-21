#pragma once

#include <bitset>
#include <utility>
#include <vector>


template<typename T_ValueType>
class CppLinearLookup
{
public:
    using ValueType = T_ValueType;

public:
    explicit
    CppLinearLookup( std::vector<ValueType> values ) :
        m_haystack( std::move( values ) )
    {}

    [[nodiscard]] std::vector<size_t>
    find( const ValueType needle,
          const size_t    distance ) const
    {
        std::vector<size_t> positions;
        for ( size_t i = 0; i < m_haystack.size(); ++i ) {
            if ( std::bitset<64>( m_haystack[i] ^ needle ).count() <= distance ) {
                positions.push_back( i );
            }
        }
        return positions;
    }

    size_t
    size() const
    {
        return m_haystack.size();
    }

private:
    std::vector<ValueType> m_haystack;
};
