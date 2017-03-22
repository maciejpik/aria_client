#include "robotManager.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>

robotManager::robotManager( int* argc, char** argv) :
    parser( argc, argv ), clientConnector( &parser ),
    keyHandler()
{
    Aria::init();

    parser.addDefaultArgument("-host 10.0.126.32");
//    parser.addDefaultArgument("-host 127.0.0.1");
    parser.loadDefaultArguments();

    try
    {
        if( !Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed() )
            throw std::runtime_error( std::string("Could not parse arguments.") );

        if( !clientConnector.connectClient( &client ) )
            throw std::runtime_error( std::string("Could not connect to the server: %s",
                                                  client.getHost() ) );

        client.setRobotName( client.getHost() );
    }
    catch( std::exception &e )
    {
        printf("Client connection build problem: %s", e.what() );

        Aria::exit();
        my_isClienRunning = false;
    }

    my_requestsHandler = new requestsHandler( &client );

    Aria::setKeyHandler( &keyHandler );
    my_steeringManager = new steeringManager( &client, &keyHandler );
    my_cameraManager = new cameraManager( &client );

    client.runAsync();

    my_isClienRunning = true;

    //client.logDataList();
    //client.findCommandFromName("listCommands");
}

/*
robotManager::robotManager()
{
    Aria::init();

    int argc = 0;
    char** argv = NULL;

    robotManager::_initializeRobotManager( argc, argv );
}
*/

robotManager::~robotManager()
{
    my_isClienRunning = false;
    client.disconnect();
    Aria::exit();
}

bool robotManager::client_getRunningWithLock()
{
    return client.getRunningWithLock();
}

bool robotManager::isClientRunning()
{
    return my_isClienRunning;
}

robotManager::requestsHandler::requestsHandler( ArClientBase* _client) :
    my_client( _client ),
    my_functor_handle_updateNumbers(this, &robotManager::requestsHandler::handle_updateNumbers),
    my_functor_handle_getSensorList(this, &robotManager::requestsHandler::handle_getSensorList),
    my_functor_handle_getSensorCurrent(this, &robotManager::requestsHandler::handle_getSensorCurrent)
{
    // Handlers installation
    my_client->addHandler("getSensorCurrent", &my_functor_handle_getSensorCurrent);

//        Add "updateNumers" requests routine (1000 ms)
    my_client->addHandler("updateNumbers", &my_functor_handle_updateNumbers);
    my_client->request("updateNumbers", 1000);

    my_client->addHandler("getSensorList", &my_functor_handle_getSensorList);
    my_client->requestOnce("getSensorList");
}

void robotManager::requestsHandler::handle_updateNumbers(ArNetPacket* packet)
{
    my_batteryVoltage = (double) packet->bufToByte2();
    my_xPosition = (double) packet->bufToByte4();
    my_yPosition = (double) packet->bufToByte4();
    my_theta = (double) packet->bufToByte2();
    my_velocity = (double) packet->bufToByte2();
    my_rotationalVelocity = (double) packet->bufToByte2();
    packet->bufToByte2(); // Skip lateralVelocity
    my_temperatur = (double) packet->bufToByte();

    printf("%3.2f|%6.2f|%6.2f|%6.2f|%6.2f|%6.2f|%6.2f\n", my_batteryVoltage, my_xPosition,
           my_yPosition, my_theta, my_velocity, my_rotationalVelocity,
           my_temperatur);
    fflush(stdout);
}

void robotManager::requestsHandler::handle_getSensorList( ArNetPacket* packet )
{
    int numberOfSensors = (int) packet->bufToByte2();
    char sensorName[255];

    if( numberOfSensors > 0 )
    {
        memset(sensorName, 0, sizeof(sensorName));
        packet->bufToStr( sensorName, sizeof( sensorName ));
    }

    printf("SENSORS: %3d => (1) %20s\n", numberOfSensors, sensorName );
    fflush(stdout);

    // Start reading data from laser reading (or the first radar received information about)
    ArNetPacket* request_name_packet = new ArNetPacket();
    request_name_packet->strToBuf( sensorName );
    request_name_packet->finalizePacket();

    my_client->request("getSensorCurrent", 100, request_name_packet);
}

void robotManager::requestsHandler::handle_getSensorCurrent( ArNetPacket* packet )
{
    int numberOfReadings = (int) packet->bufToByte2();
    if( numberOfReadings < 0 )
        return;

    char sensorName[255];
    memset(sensorName, 0, sizeof(sensorName));
    packet->bufToStr( sensorName, sizeof( sensorName ));

    std::map< int, std::pair<int, int> > reading_laser;
    for( int i = -(numberOfReadings - 1)/ 2; i <= (numberOfReadings - 1)/ 2; i++ )
    {
        reading_laser[i] = std::make_pair( packet->bufToByte4(), packet->bufToByte4());
    }
    //printf("READING (%d): (%6d, %6d)\n", 0, reading_laser[-(numberOfReadings - 1)/ 2].first, reading_laser[0].second);
    fflush(stdout);
}

robotManager::steeringManager::steeringManager( ArClientBase *_client, ArKeyHandler *_keyHandler, bool _activateKeySteering) :
    my_keySteeringActiveStatus( false ), my_isRunningByKeys( false ),
    VEL_PERC( 50 ), my_velThrottle(0), my_rotThrottle(0),
    my_client( _client ), my_keyHandler( _keyHandler ), my_clientRatioDrive( my_client ),
    my_functor_handle_key_up( this, &robotManager::steeringManager::handle_key_up),
    my_functor_handle_key_down( this, &robotManager::steeringManager::handle_key_down),
    my_functor_handle_key_left( this, &robotManager::steeringManager::handle_key_left),
    my_functor_handle_key_right( this, &robotManager::steeringManager::handle_key_right),
    my_functor_handle_key_space( this, &robotManager::steeringManager::handle_key_space),
    my_functor_thread_checkKeys( this, &robotManager::steeringManager::thread_checkKeys)
{
    if ( _activateKeySteering )
        activateKeySteering();

    my_clientRatioDrive.unsafeDrive(); // SET UNSAFE DRIVA AS DEFAULT MODE
    //turnByAngle( 30 );

}

void robotManager::steeringManager::handle_key_up()
{
    my_velThrottle += 0.025; //@ThrottleKeyboardMode
    //my_clientRatioDrive.setTransVelRatio( VEL_PERC ); //@ThrottleKeyboardMode
}

void robotManager::steeringManager::handle_key_down()
{
    my_velThrottle -= 0.025; //@ThrottleKeyboardMode
    //my_clientRatioDrive.setTransVelRatio( -VEL_PERC ); //@ThrottleKeyboardMode
}

void robotManager::steeringManager::handle_key_left()
{
    my_rotThrottle += 0.1; //@ThrottleKeyboardMode
    //my_clientRatioDrive.setRotVelRatio( VEL_PERC ); //@ThrottleKeyboardMode
}

void robotManager::steeringManager::handle_key_right()
{
    my_rotThrottle -= 0.1; //@ThrottleKeyboardMode
    //my_clientRatioDrive.setRotVelRatio( -VEL_PERC ); //@ThrottleKeyboardMode
}

void robotManager::steeringManager::handle_key_space()
{
    my_clientRatioDrive.stop();
}

void robotManager::steeringManager::thread_checkKeys()
{
    while( true )
    {
        my_keyHandler->checkKeys();

        //@ThrottleKeyboardMode
        //This mod simulate throttle for keyboard driving. Please remove all
        //@ThrottleKeyboardMode addresses / comments to get back to default
        //setup.
        if (ArMath::fabs( my_velThrottle ) < 0.01 &&
                ArMath::fabs( my_rotThrottle ) < 0.01)
        {
            if ( my_isRunningByKeys )
            {
                my_clientRatioDrive.stop();
                my_isRunningByKeys = false;
            }
        }
        else
        {
            my_isRunningByKeys = true;

            if (my_velThrottle > 0)
                my_velThrottle -= 0.015;
            else
                my_velThrottle += 0.015;

            if (my_rotThrottle > 0)
                my_rotThrottle -= 0.025;
            else
                my_rotThrottle += 0.025;

            if (my_velThrottle < -1)
                my_velThrottle = -1;
            else if(my_velThrottle > 1)
                my_velThrottle = 1;
            if (my_rotThrottle < -1)
                my_rotThrottle = -1;
            else if (my_rotThrottle > 1)
                my_rotThrottle = 1;

            my_clientRatioDrive.setTransVelRatio( VEL_PERC * my_velThrottle);
            my_clientRatioDrive.setRotVelRatio( VEL_PERC * my_rotThrottle);
        }
        ArUtil::sleep( 100 );
    }
}

void robotManager::steeringManager::activateKeySteering()
{
    if( !my_keySteeringActiveStatus )
    {
        my_keyHandler->addKeyHandler(ArKeyHandler::UP, &my_functor_handle_key_up);
        my_keyHandler->addKeyHandler(ArKeyHandler::DOWN, &my_functor_handle_key_down);
        my_keyHandler->addKeyHandler(ArKeyHandler::LEFT, &my_functor_handle_key_left);
        my_keyHandler->addKeyHandler(ArKeyHandler::RIGHT, &my_functor_handle_key_right);
        my_keyHandler->addKeyHandler(ArKeyHandler::SPACE, &my_functor_handle_key_space);

        my_thread_checkKeys.create( &my_functor_thread_checkKeys ); // CREATE THREAD FOR CHECKING KEYS

        my_keySteeringActiveStatus = true;
    }
    return;
}

void robotManager::steeringManager::deactivateKeySteering()
{
    if ( my_keySteeringActiveStatus )
    {
        my_thread_checkKeys.cancel();

        my_keyHandler->remKeyHandler(ArKeyHandler::UP);
        my_keyHandler->remKeyHandler(ArKeyHandler::DOWN);
        my_keyHandler->remKeyHandler(ArKeyHandler::LEFT);
        my_keyHandler->remKeyHandler(ArKeyHandler::RIGHT);
        my_keyHandler->remKeyHandler(ArKeyHandler::SPACE);

        my_keySteeringActiveStatus = false;
    }
    return;
}

void robotManager::steeringManager::moveDistance( double distance_mm )
{
    handle_jogModeRequests(0, distance_mm);
    return;
}

void robotManager::steeringManager::turnByAngle( double angle_deg )
{
    handle_jogModeRequests(1, angle_deg);
    return;
}

void robotManager::steeringManager::turnToHeading( double angle_deg )
{
    handle_jogModeRequests(2, angle_deg);
    return;
}

void robotManager::steeringManager::handle_jogModeRequests(int i, double value )
{
    std::string mode;
    switch ( i )
    {
    case 0:
        mode = "moveDist";
        break;

    case 1:
        mode = "turnByAngle";
        break;

    case 2:
        mode = "turnToHeading";
        break;

    default:
        return;
    }
    ArNetPacket* packet = new ArNetPacket();
    packet->doubleToBuf( value );
    packet->finalizePacket();

    my_client->requestOnce(mode.c_str(), packet);
    return;
}

robotManager::cameraManager::cameraManager( ArClientBase* _client ) :
    my_recordToFolder( false ), my_client( _client ),
    my_functor_handle_getCameraList(this, &robotManager::cameraManager::handle_getCameraList),
    my_functor_handle_snapshot(this, &robotManager::cameraManager::handle_snapshot),
    my_functor_handle_getCameraInfoCamera_1(this, &robotManager::cameraManager::handle_getCameraInfoCamera_1),
    my_functor_hanlde_getCameraDataCamera_1(this, &robotManager::cameraManager::handle_getCameraDataCamera_1)
    {
//        Something is wrong with this request. Please check in header file.
//        my_client->addHandler("getCameraList", &my_functor_handle_getCameraList);
//        my_client->requestOnce("getCameraList");

        my_client->addHandler("sendVideo", &my_functor_handle_snapshot);
        my_client->request("sendVideo", 1000);

        my_client->addHandler("getCameraInfoCamera_1", &my_functor_handle_getCameraInfoCamera_1);
        my_client->request("getCameraInfoCamera_1", 1000);

        my_client->addHandler("getCameraDataCamera_1", &my_functor_hanlde_getCameraDataCamera_1);
        my_client->request("getCameraDataCamera_1", 1000);

        handle_setCameraAbsCamera_1(1000, 1000, 0);
        resetPosition();
    }

void robotManager::cameraManager::handle_getCameraList( ArNetPacket* packet )
{
    int numberOfCameras = (int) packet->bufToByte2();
    if( numberOfCameras < 1 )
    {
        printf("getCameraList handler: Could not find any camera.\n"); fflush(stdout);
        return;
    }

    // This function works only for one camera (it saves settings of the 1st
    // described camera in the packet).
    memset(my_cameraName, 0, sizeof(my_cameraName));
    memset(my_cameraType, 0, sizeof(my_cameraType));
    memset(my_cameraNameForUserDisplay, 0, sizeof(my_cameraNameForUserDisplay));
    memset(my_cameraTypeForUserDisplay, 0, sizeof(my_cameraTypeForUserDisplay));

    packet->bufToStr( my_cameraName, sizeof( my_cameraName ));
    packet->bufToStr( my_cameraType, sizeof( my_cameraType ));
    packet->bufToStr( my_cameraNameForUserDisplay, sizeof( my_cameraNameForUserDisplay ));
    packet->bufToStr( my_cameraTypeForUserDisplay, sizeof( my_cameraTypeForUserDisplay ));

    int numberOfCommands = packet->bufToByte2();

    char genericName[255], commandName[255];
    int commandFrequency = 0;
    for(int i = 0; i < numberOfCommands; i++) {
        memset( genericName, 0, sizeof(genericName));
        memset( commandName, 0, sizeof( commandName));

        packet->bufToStr( genericName, sizeof(genericName));
        packet->bufToStr( commandName, sizeof(commandName));
        commandFrequency = packet->bufToByte4();

        printf("Command %d -----\n%s\n%s\nFrequency: %d",
               i, genericName, commandName, commandFrequency );
        fflush(stdout);
    }

    // Further the packet contains information about accepted
    // parameters so it would need another loop implementation
    // to iterate through the packet's elements - this information
    // is left here alone.
    return;
}

void robotManager::cameraManager::handle_snapshot( ArNetPacket* packet ) {
    // Meta data variables:
    int width = (int) packet->bufToByte2();
    int height = (int) packet->bufToByte2();

    // Calculate how much should we read to obtain image
    int toRead = (int) packet->getDataLength() - (int) packet->getDataReadLength();
    unsigned char image[toRead];
    packet->bufToData( image, toRead );

    if( my_recordToFolder )
        recordFrame( image, toRead );

//    FILE* f = fopen("image_data.txt", "wb");
//    fwrite(image, toRead, 1, fp2);
////    for(int i = 0; i < toRead; i++)
////        fprintf(f, "%c\n", image[i]);
//    fclose(f);

    printf("Snap: %d | %d | %d\n", width, height, toRead); fflush(stdout);
}

void robotManager::cameraManager::handle_getCameraInfoCamera_1( ArNetPacket* packet ) {
    my_camera_minPan = (int) packet->bufToByte2();
    my_camera_maxPan = (int) packet->bufToByte2();
    my_camera_minTilt = (int) packet->bufToByte2();
    my_camera_maxTilt = (int) packet->bufToByte2();
    my_camera_minZoom = (int) packet->bufToByte2();
    my_camera_maxZoom = (int) packet->bufToByte2();
    my_camera_isZoomAvailable = (bool) packet->bufToByte();

    printf("## Camera parameters:\n\
# min Pan: %d\n\
# max Pan: %d\n\
# min Tilt: %d\n\
# max Tilt: %d\n\
# min Zoom: %d\n\
# max Zoom: %d\n\
##\n", my_camera_minPan, my_camera_maxPan,
           my_camera_minTilt, my_camera_maxTilt,
           my_camera_minZoom, my_camera_maxZoom);
}

void robotManager::cameraManager::handle_getCameraDataCamera_1( ArNetPacket* packet ) {
    my_camera_pan = (int) packet->bufToByte2();
    my_camera_tilt = (int) packet->bufToByte2();
    my_camera_zoom = (int) packet->bufToByte2();

    printf("## Camera position:\n\
# Pan: %d\n\
# Tilt: %d\n\
# Zoom: %d\n\
##\n", my_camera_pan, my_camera_tilt, my_camera_zoom);
}

void robotManager::cameraManager::handle_setCameraAbsCamera_1(int pan, int tilt, int zoom) {
    ArNetPacket* packet = new ArNetPacket();
    packet->byte2ToBuf(pan);
    packet->byte2ToBuf(tilt);
    packet->byte2ToBuf(zoom);
    packet->finalizePacket();
    my_client->requestOnce("setCameraAbsCamera_1", packet);
}

void robotManager::cameraManager::handle_setCameraRelCamera_1(int plus_pan, int plus_tilt, int plus_zoom) {
    ArNetPacket* packet = new ArNetPacket();
    packet->byte2ToBuf(plus_pan);
    packet->byte2ToBuf(plus_tilt);
    packet->byte2ToBuf(plus_zoom);
    packet->finalizePacket();
    my_client->requestOnce("setCameraRelCamera_1", packet);
}

void robotManager::cameraManager::resetPosition() {
    handle_setCameraAbsCamera_1(0, 0, 0);
}

void robotManager::cameraManager::recordFrame(unsigned char* image, int length_of_image) {
//    Please be careful, this image is encoded in jpeg format
    std::ostringstream stream_converter;
    stream_converter << my_frame_number;
    std::string temp_string = stream_converter.str();
    std::string filename;
    filename = std::string("video_record/")
        + std::string( my_filename_length - temp_string.length(), '0')
        + temp_string + my_file_extension;

    FILE *image_file;
    image_file = fopen(filename.c_str(), "wb");
    fwrite(image, length_of_image, 1, image_file);
    fclose(image_file);

    my_frame_number++;
}

void robotManager::cameraManager::startRecording() {
    my_filename_length = 10;
    my_file_extension = std::string(".jpg");
    my_frame_number = 0;
    my_recordToFolder = true;
}

void robotManager::cameraManager::stopRecording() {
    my_recordToFolder = false;
}
