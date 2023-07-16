#include <cstdint>
#include <iostream>
#include <sstream>

int main()
{
    uint8_t i = 3;
    std::stringstream buffer;
    buffer.write( reinterpret_cast<char*>( &i ), 1 );
    std::cerr << buffer.good() << "\n";

    buffer.read( reinterpret_cast<char*>( &i ), 1 );
    std::cerr << buffer.good() << "\n";

    buffer.read( reinterpret_cast<char*>( &i ), 1 );
    std::cerr << buffer.good() << "\n";

    return 0;
}
