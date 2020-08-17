#define CATCH_CONFIG_RUNNER
#include <Catch/catch.hpp>
#include "constants.h"

libconfig::Config cfg = libconfig::Config();

int main( int argc, char* argv[] ) 
{
    cfg.readFile(TEST_CONSTANTS_FILE);

    int result = Catch::Session().run( argc, argv );

    return result;
}