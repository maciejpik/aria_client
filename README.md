# Aria client for Pioneer robot

This piece of software along with [aria_server](https://github.com/maciejpik/aria_server) is intended to help in integrating Pioneer robot and its camera with computer vision software which algorithms might be computationally expensive thus not usable on Pioneer's onboard computer.

Core of the software is *robotManager* class which is a container for four nested classes *requestsHandler, keyHandlerMaster, cameraManager, steeringManager* each being another smaller manager. These nested classes are responsible respectively for:
  * requesting and receiving general information about the robot
  * handling key strokes on client side
  * receiving camera stream from robot and steering its camera
  * steering the robot

### Beginning
To reach properties and methods of nested classes one can use *robotManager* properties which are in fact instances of mentioned nested classes.
```cpp
robotManager rManager( &argc, argv );

rManager.requests->methodOfClass_requestsHandler();
rManager.steering->methodOfClass_steeringManager();
rManager.camera->methodOfClass_cameraManager();
rManager.keyHandler->methodOfClass_keyHandlerMaster();
```

### Verbose mode
To enable verbose mode in particular manager use *enableVerboseMode()* method.
```cpp
rManager.camera->enableVerboseMode();
```

### Steering Pioneer and its camera with client's keyboard
In order to activate robot's steering or camera's steering with keyboard use following methods.
  * ← → ↑ ↓ - steering robot
  * *w s a d* - camera pan / tilt
  * *r f* - camer zoom in / out

```cpp
rManager.keyHandler->startKeyMaster(); // Activate key strokes handling
rManager.steering->activateKeySteering(); // Activate robot's steering
rManager.camera->activateCameraSteering(); // Activate camera steering
```

### Camera stream to OpenCV
To get live stream from Pioneer's camera to OpenCV object you can use following code.
```cpp
cv::namedWindow( "Stream", CV_WINDOW_AUTOSIZE );
std::pair<unsigned char*, int> imageData;
while( true )
{
    imageData = rManager.camera->getSendVideoFrame();
    std::vector<unsigned char> buffer( imageData.first, imageData.first + imageData.second );
    cv::Mat image = imdecode(buffer, cv::IMREAD_ANYCOLOR);
    if(image.empty())
        return 0;
    cv::imshow("Stream", image);
    cv::waitKey( myStream->getSynchroTime_ums() / 1000 );
}
```