#ifndef ROBOTMANAGER_H_INCLUDED
#define ROBOTMANAGER_H_INCLUDED

#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientRatioDrive.h"

class robotManager
{
public:
    robotManager(int* argc, char** argv);
    ~robotManager();

    bool client_getRunningWithLock();
    bool isClientRunning();

private:
    class requestsHandler
    {
    public:
        requestsHandler( ArClientBase *_client );

//    CALLBACKS FUNCTIONS
        void handle_updateNumbers( ArNetPacket *packet );
        void handle_getSensorList( ArNetPacket *packet );
        void handle_getSensorCurrent( ArNetPacket *packet );

    protected:
        double my_batteryVoltage,
               my_xPosition, my_yPosition,
               my_theta, my_velocity,
               my_rotationalVelocity,
               my_temperatur;

        ArClientBase* my_client;

//    CALLBACKS FUNCTORS
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_updateNumbers;
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_getSensorList;
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_getSensorCurrent;

        // HELP FUNCTIONS
    };

    class steeringManager
    {
    public:
        steeringManager( ArClientBase *_client, ArKeyHandler *_keyHandler, bool _activateKeySteering = true);

        void moveDistance( double distance_mm );
        void turnByAngle( double angle_deg );
        void turnToHeading( double angle_deg );

    private:
        bool my_keySteeringActiveStatus,
        my_isRunningByKeys;

        const int VEL_PERC;

        double my_velThrottle, my_rotThrottle;

    protected:
        ArClientBase* my_client;
        ArKeyHandler* my_keyHandler;
        ArClientRatioDrive my_clientRatioDrive;

        ArThread my_thread_checkKeys; //ATTENTION! THIS CLASS CREATS ITS OWN THREAD

        void activateKeySteering(void);
        void deactivateKeySteering(void);

        void handle_jogModeRequests( int type, double value );

        // CALLBACKS FUNCTIONS
        void handle_key_up(void);
        void handle_key_down(void);
        void handle_key_left(void);
        void handle_key_right(void);
        void handle_key_space(void);
        void thread_checkKeys(void);

        // CALLBACKS FUNCTORS
        ArFunctorC<steeringManager> my_functor_handle_key_up;
        ArFunctorC<steeringManager> my_functor_handle_key_down;
        ArFunctorC<steeringManager> my_functor_handle_key_left;
        ArFunctorC<steeringManager> my_functor_handle_key_right;
        ArFunctorC<steeringManager> my_functor_handle_key_space;
        ArFunctorC<steeringManager> my_functor_thread_checkKeys;
    };

    class cameraManager
    {
    public:
        cameraManager( ArClientBase* _client );

        // Camera steering
        void resetPosition();

    private:
        char my_cameraName[255], my_cameraType[255],
        my_cameraNameForUserDisplay[255],
        my_cameraTypeForUserDisplay[255];

        // Camera parameters
        int my_camera_minPan, my_camera_maxPan;
        int my_camera_minTilt, my_camera_maxTilt;
        int my_camera_minZoom, my_camera_maxZoom;
        bool my_camera_isZoomAvailable;

        // Current camera position
        int my_camera_pan, my_camera_tilt,
        my_camera_zoom;

    protected:
        ArClientBase* my_client;

        void handle_setCameraAbsCamera_1(int pan, int tilt, int zoom);
        void handle_setCameraRelCamera_1(int plus_pan, int plus_tilt, int plus_zoom);

        // CALLBACKS FUNCTIONS
        void handle_getCameraList( ArNetPacket* packet ); // This function does
        // not work properly right now with current Pioneer configuration. To
        // consider if I still needs it.
        void handle_snapshot( ArNetPacket* packet);
        void handle_getCameraInfoCamera_1( ArNetPacket* packet);
        void handle_getCameraDataCamera_1( ArNetPacket* packet);

        // CALBACKS FUNCTORS
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_getCameraList;
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_snapshot;
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_getCameraInfoCamera_1;
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_hanlde_getCameraDataCamera_1;
    };

private:
    ArArgumentParser parser;
    ArClientBase client;
    ArClientSimpleConnector clientConnector;
    ArKeyHandler keyHandler;

    requestsHandler* my_requestsHandler;
    steeringManager* my_steeringManager;
    cameraManager* my_cameraManager;

    bool my_isClienRunning;

};

#endif // ROBOTMANAGER_H_INCLUDED
