#include <iostream>
#include <stdexcept>
#include <string>
#include <cstdio>

#include "robotManager.h"

// DEVELOPMENT LIBRARIES
#include <unistd.h>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

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

    cv::namedWindow( "Stream", CV_WINDOW_AUTOSIZE );
    std::pair<unsigned char*, int> imageData;
    while( _robotManager.client_getRunningWithLock() )
    {
        imageData = _robotManager.camera->getSendVideoFrame();
        std::vector<unsigned char> buffer( imageData.first, imageData.first + imageData.second );
        cv::Mat image = imdecode(buffer, cv::IMREAD_ANYCOLOR);
        // cv::Mat image = imdecode(buffer, cv::IMREAD_GRAYSCALE);
        if(image.empty())
            return 0;
        cv::imshow("Stream", image);
        cv::waitKey( _robotManager.camera->getSynchroTime_ums() / 1000 );
    }
    return 0;
}
