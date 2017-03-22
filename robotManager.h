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
        void enableVerboseMode();

    protected:
        double my_batteryVoltage,
               my_xPosition, my_yPosition,
               my_theta, my_velocity,
               my_rotationalVelocity,
               my_temperatur;

        ArClientBase* my_client;

        bool my_verboseMode;

//    CALLBACKS FUNCTIONS
        void handle_updateNumbers( ArNetPacket *packet );
        void handle_getSensorList( ArNetPacket *packet );
        void handle_getSensorCurrent( ArNetPacket *packet );

//    CALLBACKS FUNCTORS
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_updateNumbers;
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_getSensorList;
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_getSensorCurrent;
    };

    class cameraManager
    {
    public:
        cameraManager( ArClientBase* _client, ArKeyHandler* _keyHandler );

        // Camera steering
        void resetPosition();
        void handle_setCameraAbsCamera_1(int pan, int tilt, int zoom);
        void handle_setCameraRelCamera_1(int plus_pan, int plus_tilt, int plus_zoom);

        // Key camera steering
        void activateCameraSteering();
        void deactivateCameraSteering();

        // Send video
        int getSendVideoDelay();
        std::pair<unsigned char*, int> getSendVideoFrame();

        void enableVerboseMode();

    private:
        char my_cameraName[255], my_cameraType[255],
             my_cameraNameForUserDisplay[255],
             my_cameraTypeForUserDisplay[255];

        bool my_verboseMode;

        // Camera parameters
        int my_camera_minPan, my_camera_maxPan;
        int my_camera_minTilt, my_camera_maxTilt;
        int my_camera_minZoom, my_camera_maxZoom;
        bool my_camera_isZoomAvailable;

        // Current camera position
        int my_camera_pan, my_camera_tilt,
            my_camera_zoom;

        // Send video
        unsigned char my_lastSnap[38400];
        int my_lastSnapSize, my_sendVideoDelay;
        bool my_video_mutexOn;

        // Frame recording variables
        bool my_recordToFolder;
        int my_frame_number, my_filename_length;
        std::string my_file_extension;

        // Camera key steering
        bool my_cameraSteeringActiveStatus;

    protected:
        ArClientBase* my_client;
        ArKeyHandler* my_keyHandler;

        ArThread my_thread_checkKeys; //ATTENTION! THIS CLASS CREATS ITS OWN THREAD

        void recordFrame(unsigned char* image, int length_of_image );
        void startRecording();
        void stopRecording();

        // CALLBACKS FUNCTIONS
        void handle_getCameraList( ArNetPacket* packet ); // This function does
        // not work properly right now with current Pioneer configuration. To
        // consider if I still needs it.
        void handle_snapshot( ArNetPacket* packet);
        void handle_getCameraInfoCamera_1( ArNetPacket* packet);
        void handle_getCameraDataCamera_1( ArNetPacket* packet);
        // Key handling (S)
        void handle_key_w(void);
        void handle_key_s(void);
        void handle_key_a(void);
        void handle_key_d(void);
        void handle_key_r(void);
        void handle_key_f(void);
        void thread_checkKeys(void);
        // Key handling (E)

        // CALBACKS FUNCTORS
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_getCameraList;
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_snapshot;
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_getCameraInfoCamera_1;
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_hanlde_getCameraDataCamera_1;
        // Key handling (S)
        ArFunctorC<cameraManager> my_functor_handle_key_w;
        ArFunctorC<cameraManager> my_functor_handle_key_s;
        ArFunctorC<cameraManager> my_functor_handle_key_a;
        ArFunctorC<cameraManager> my_functor_handle_key_d;
        ArFunctorC<cameraManager> my_functor_handle_key_r;
        ArFunctorC<cameraManager> my_functor_handle_key_f;
        ArFunctorC<cameraManager> my_functor_thread_checkKeys;
        // Key handling (E)
    };

    class steeringManager
    {
    public:
        steeringManager( ArClientBase *_client, ArKeyHandler *_keyHandler,
                         bool _activateKeySteering = true);

        void moveDistance( double distance_mm );
        void turnByAngle( double angle_deg );
        void turnToHeading( double angle_deg );

        void enableVelocitySteering();
        void enableDistSteering();

        void enableVerboseMode();

    private:
        bool my_verboseMode, my_keySteeringActiveStatus,
             my_isRunningByKeys, my_isVelocitySteering;

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

private:
    ArArgumentParser parser;
    ArClientBase client;
    ArClientSimpleConnector clientConnector;
    ArKeyHandler keyHandler;

    bool my_isClienRunning;

public:
    requestsHandler* requests;
    steeringManager* steering;
    cameraManager* camera;
};

#endif // ROBOTMANAGER_H_INCLUDED
