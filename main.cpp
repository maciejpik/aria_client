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
    robotManager _robotManager( &argc, argv, "10.0.126.32" );

    // Enable verbose mode
    _robotManager.requests->enableVerboseMode();
    _robotManager.steering->enableVerboseMode();
    _robotManager.camera->enableVerboseMode();

    // Enable key handling
    _robotManager.keyHandler->startKeyMaster();

    _robotManager.camera->activateCameraSteering();

    while ( _robotManager.client_getRunningWithLock() )
    {
        printf("Dzialam...\n");
        fflush(stdout);
        usleep( 10000000 );
    }
    return 0;
}
