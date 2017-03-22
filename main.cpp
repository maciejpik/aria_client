#include <iostream>
#include <stdexcept>
#include <string>
#include <cstdio>

#include "robotManager.h"

// DEVELOPMENT LIBRARIES
#include <unistd.h>
#include <signal.h>

using namespace std;


int main(int argc, char **argv)
{
    robotManager _robotManager( &argc, argv );

    while ( _robotManager.client_getRunningWithLock() )
    {
        printf("Dzialam...\n");
        fflush(stdout);
        usleep( 10000000 );
    }
    return 0;
}
