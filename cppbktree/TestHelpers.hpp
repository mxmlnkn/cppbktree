#pragma once

#include <iostream>
#include <string>
#include <vector>


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
        if constexpr ( std::is_same_v<uint8_t, T> ) {
            out << static_cast<unsigned int>( x );
        } else if constexpr ( std::is_same_v<int8_t, T> ) {
            out << static_cast<int>( x );
        } else {
            out << x;
        }
        out.copyfmt( ioState );
    }

    out << " }";
    return out;
}


int gnTests = 0;  // NOLINT
int gnTestErrors = 0;  // NOLINT


template<typename A,
         typename B>
void
requireEqual( const A&  a,
              const B&  b,
              const int line )
{
    ++gnTests;
    if ( a != b ) {
        ++gnTestErrors;
        std::cerr << "[FAIL on line " << line << "] " << a << " != " << b << "\n";
    }
}


void
require( bool               condition,
         std::string const& conditionString,
         int                line )
{
    ++gnTests;
    if ( !condition ) {
        ++gnTestErrors;
        std::cerr << "[FAIL on line " << line << "] " << conditionString << "\n";
    }
}


#define REQUIRE_EQUAL( a, b ) requireEqual( a, b, __LINE__ )  // NOLINT
#define REQUIRE( condition ) require( condition, #condition, __LINE__ )  // NOLINT
