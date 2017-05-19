#ifndef ROBOTMANAGER_H_INCLUDED
#define ROBOTMANAGER_H_INCLUDED

#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientRatioDrive.h"

/** \brief Główna klasa odpowiadająca za komunikację z robotem
 *
 * Klasa \c robotManager reprezentuje obiekt, przez który przeprowadzana jest
 * cała komunikacja z robotem (serwerem). Użytkownik inicjuje w swoim programie
 * połączenie z robotem poprzez stworzenie obiektu klasy \c robotManager i przekazanie
 * w konstruktorze klasy komend uszczegóławiających warunki połączenia z serwerem
 * wraz z adresem IP robota.
 *
 * W klasie \c robotManager zagnieżdżone są klasy odpowiedzialne za poszczególne
 * funkcje robota. Użytkownik, aby korzystać z wyżej wymienionych zagnieżdżonych
 * klas, może użyć następujących pól klasy \c robotManager:
 * \li \c requests - odbieranie informacji dot. robota, np.: wskazań lasera
 * \li \c steering - sterowanie robotem
 * \li \c camera - obsługa kamery
 * \li \c keyHandler - obsługa klawiatury w programie użytkownika
 */
class robotManager
{
public:
    /** \brief Konstruktor klasy \c robotManager
     *
     * \param argc int* - liczba dodatkowych parametrów
     * \param argv char** - treść dodatkowych parametrów (polecenia dostępne w bibliotece Aria)
     * \param ipAddress std::string - adres IP robota
     *
     */
    robotManager(int* argc, char** argv, std::string ipAddress);
    /** \brief Destruktor klasy \c robotManager
     *
     *
     */
    ~robotManager();

    /** \brief Przekazuje wyjście metody ArClientBase.getRunningWithLock()
     *
     * \return bool - wyjście metody ArClientBase.getRunningWithLock()
     *
     */
    bool client_getRunningWithLock();
    /** \brief Sprawdza czy klient robota działa
     *
     * \return bool - \b True, jeśli klient robota działa
     *
     */
    bool isClientRunning();
    /** \brief Zmienia poziom logowania w bibliotece Aria do ArLog::Terse
     *
     * \return void
     *
     */
    void disableNativeAriaLogging();

private:
    /** \brief Odbieranie informacji dotyczących robota
     *
     * Klasa pozwala na odbieranie informacji dotyczących robota, np.: pomiar z
     * dalmierza laserowego, prędkość kątowa.
     */
    class requestsHandler
    {
    public:
        /** \brief Konstruktor klasy \c robotManager::requestsHandler
         *
         * \param _client ArClientBase* - wskaźnik do obiektu klienta Aria
         *
         */
        requestsHandler( ArClientBase *_client );
        /** \brief Włącza wyświetlanie dotakowych informacji
         *
         * \return void
         *
         */
        void enableVerboseMode();
        /** \brief Rozpoczyna pobieranie danych z dalmierza laserowego
         *
         * \return bool - \b True, jeżeli udało się połączyć z dalmierzem
         *
         */
        bool startReadingLaser();

        // Getters for position information
        /** \brief Zwraca współrzędną \c x robota w układzie współrzędnych robota
         *
         * \return double - współrzędna \c x robota
         *
         */
        double get_xPosition();
        /** \brief Zwraca współrzędną \c y robota w układzie współrzędnych robota
         *
         * \return double - współrzędna \c y robota
         *
         */
        double get_yPosition();
        /** \brief Zwraca kąt obrotu robota w układzie współrzędnych robota
         *
         * \return double - kąt obrotu robota
         *
         */
        double get_theta();
        /** \brief Zwraca prędkość translacyjną robota
         *
         * \return double - prędkość robota
         *
         */
        double get_velocity();
        /** \brief Zwraca prędkość kątową robota
         *
         * \return double - prędkość kątowa robota
         *
         */
        double get_rotationalVelocity();

        /** \brief Zwraca listę wszystkich dostępnych czujników pomiarowych w robocie
         *
         * \return std::vector<std::string> - wektor nazw czujników pomiarowych dostępnych w robocie
         *
         */
        std::vector<std::string> get_sensorsVector();
        /** \brief Zwraca ostatni pomiar z dalmierza laserowego
         *
         * \todo Przetworzenie pomiarów z postaci \c RAW do innej, wygodniejszej w użyciu postaci, np.: kąt - odległość
         *
         * \return std::map< int, std::pair<int, int>> - pomiar w postaci \c RAW (patrz dokumentacja biblioteki Aria)
         *
         */
        std::map< int, std::pair<int, int> > get_laserReading();

    private:
        double my_batteryVoltage,
               my_xPosition, my_yPosition,
               my_theta, my_velocity,
               my_rotationalVelocity,
               my_temperatur;/**< Podstawowe informacje o stanie robota */

        ArClientBase* my_client;/**< Wskaźnik do obiektu klienta Aria */

        bool my_verboseMode;/**< Stan opcji wyświetlania dodatkowych informacji */

        std::vector<std::string> my_sensorsVector;/**< Lista nazw dostępnych sensorów w robocie */
        std::map< int, std::pair<int, int> > my_laserReading;/**< Ostatnio odczytany pomiar z dalmierza laserowego */

        // CALLBACKS FUNCTIONS
        void handle_updateNumbers( ArNetPacket *packet );/**< \brief callback polecenia \c updateNumbers */
        void handle_getSensorList( ArNetPacket *packet );/**< \brief callback polecenia \c getSensorList */
        void handle_getSensorCurrent( ArNetPacket *packet );/**< \brief callback polecenia \c getSensorCurrent */

        // CALLBACKS FUNCTORS
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_updateNumbers;/**< functor dla polecenia \c updateNumbers */
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_getSensorList;/**< functor dla polecenia \c getSensorList */
        ArFunctor1C<requestsHandler, ArNetPacket*> my_functor_handle_getSensorCurrent;/**< functor dla polecenia \c getSensorList */
    };

    /** \brief Zarządzanie klawiaturą
     *
     * Klasa ta obsługuje wciśnięcia klawiszy - użytkownik biblioteki
     * \c robotManager może aktywować działania tej klasy poprzez wywołanie metody
     * \c startKeyMaster().
     *
     * Dodatkowo, możliwe jest przekazanie funkcji typu \c callback, które zostaną
     * wywoałne podczas każdego sprawdzenia przez klasą stanu klawiszy - metody
     * \c addCallback(ArFunction \c *func) oraz \c removeCallback(ArFunction \c *func).
     *
     * Możliwa jest także symulacja wciśnięcia danego klawisza poprzez podanie kodu
     * klawisza do funckji \c pressKey(int \c key).
     *
     * Klasa ta (sprawdzanie stanu klawiszy) działa we własnym wątku.
     */
    class keyHandlerMaster: public ArKeyHandler
    {
    public:
        /** \brief Konstruktor klasy \c robotManager::keyHandlerMaster
         *
         * Wszystkie przekazane parametry w tym konstruktorze mają wartości domyślne
         * zgodne z dokumentacją biblioteki Aria - przekazywane są do konstruktora
         * klasy ArKeyHandler.
         */
        keyHandlerMaster(bool blocking=false, bool addAriaExitCB=true,
                         FILE* stream=NULL, bool takeKeysInConstructor=true);
        /** \brief Destrukator klasy \c robotManager::keyHandlerMaster
         *
         *
         */
        ~keyHandlerMaster();

        /** \brief Dodaje \c callback do pętli sprawdzającej stan klawiszy
         *
         * \param func ArFunctor* - wskaźnik na \c functor obsługujący zadaną funkcję typu \c callback
         * \return void
         *
         */
        void addCallback(ArFunctor *func);
        /** \brief Usuwa \c callback z pętli sprawdzającej stan klawiszy
         *
         * \param func ArFunctor* - wskaźnik na \c functor obsługujący zadaną funkcję typu \c callback
         * \return void
         *
         */
        void removeCallback(ArFunctor *func);

        /** \brief Rozpoczyna obsługę klawiatury
         *
         * Metoda ta rozpoczyna działanie pętli sprawdzającej stan klawiszy.
         *
         * \return void
         *
         */
        void startKeyMaster();
        /** \brief Zatrzymuje obsługę klawiszy
         *
         * \return void
         *
         */
        void stopKeyMaster();

        /** \brief Symuluje wciśnięce danego klawisza
         *
         * \param key int - kod symulowanego klawisza
         * \return void
         *
         */
        void pressKey(int key);
        /** \brief Wywołuje przekazane obiektowi tej klasy funkcje typu \c callback
         *
         * Funkcje typu \c callback dodawane są metodą \c addCallback(ArFunction \c *func)
         *
         * \return void
         *
         */
        void invokeCallbacks();

    private:
        ArThread my_thread_checkKeys;/**< handler wątku sprawdzania stanu klawiszy */
        std::vector<ArFunctor*> my_callbacksVector;/**< zbiór funkcji typu \c callback */

        int findElementIndex(ArFunctor *func);/**< \brief Zwraca pozycję danego funkctora w wektorze \c my_callbacksVector  */

        // CALLBACKS FUNCTIONS
        void thread_checkKeys(void);/**< wątek sprawdzania stanu klawiszy */

        // CALLBACKS FUNCTORS
        ArFunctorC<keyHandlerMaster> my_functor_thread_checkKeys;/**< functor do metody \c thread_checkKeys() (wątku sprawdzania stanu klawiszy) */
    };

    /** \brief Obsługa kamery
     *
     * Klasa służy do obsługi kamery dostępnej w robocie Pioneer. Udostępnia on użytkownikowi
     * metody służące do sterowania kamerą oraz odczytywania z niej obrazu.
     *
     * Sterowanie kamerą możliwe jest po aktywacji metodą \c activateCameraSteering()
     * następującymi klawiszami:
     * \li a / d - obrót
     * \li w / s - pochylenie
     * \li r / f - zbliżenie / oddalenie
     */
    class cameraManager
    {
    public:
        /** \brief Konstruktor klasy \c robotManager::cameraManager
         *
         * \param _client ArClientBase* - wskaźnik do klienta Aria
         * \param _keyHandler keyHandlerMaster* - wskaźnik do obiektu obsługującego zdarzenia związane z klawiaturą
         *
         */
        cameraManager( ArClientBase* _client, keyHandlerMaster* _keyHandler );

        // Camera steering
        /** \brief Resetuje ustawianie kamery do położenia początkowego.
         *
         * Domyślne położenie początkowe to (obrót, pochylenie, zoom) = (0, 0, 0),
         * które równoważne jest temu, że kamera skierowana jest na wprost i zoom
         * przyjmuje minimalną wartość.
         *
         * \return void
         *
         */
        void resetPosition();
        /** \brief Ustawia kamerę dla obrotu, pochylenia i zbliżenia
         *
         * \param pan int - obrót
         * \param tilt int - pochylenie
         * \param zoom int - zbliżenie
         * \return void
         *
         */
        void handle_setCameraAbsCamera_1(int pan, int tilt, int zoom);
        /** \brief Przestawia kamerę o zadaną wartość dla obrotu, pochylenia i zbliżenia
         *
         * \param plus_pan int - obrót
         * \param plus_tilt int - pochylenie
         * \param plus_zoom int - zbliżenie
         * \return void
         *
         */
        void handle_setCameraRelCamera_1(int plus_pan, int plus_tilt, int plus_zoom);

        // Key camera steering
        /** \brief Aktywuje sterowanie kamerą za pomocą klawiatury
         *
         * Sterowanie odbywa się za pomocą następujących klawiszy:
         * \li a / d - obrót
         * \li w / s - pochylenie
         * \li r / f - zbliżenie / oddalenie
         *
         * \return void
         *
         */
        void activateCameraSteering();
        /** \brief Wyłącza sterowanie kamerą za pomocą klawiatury
         *
         * \return void
         *
         */
        void deactivateCameraSteering();

        // Send video
        /** \brief Zwraca odstęp czasowy pomiędzy kolejnymi klatkami w strumieniu z kamery
         *
         * \return int - odstęp czasowy pomiędzy kolejnymi klatkami w \c ms
         *
         */
        int getSendVideoDelay();
        /** \brief Zwraca odstęp czasowy pomiędzy kolejnymi klatkami w strumieniu z kamery
         *
         * \return int - odtęp czasowy pomiędzy kolejnymi klatkami w \c ums
         *
         */
        int getSynchroTime_ums();
        /** \brief Zwraca ostatnio pobraną klatkę obrazu z kamery
         *
         * Para, która zwraca jest przez funkcję reprezentuje blob obrazu z kamery wraz z jego długością.
         * Zaproponowany format zwracanej wielkości jest bardzo łatwy do zintegrowania z OpenCV, np.:
         * \code
         * std::pair<unsigned char*, int> imageData;
         * imageData = rManager.camera->getSendVideoFrame();
         * std::vector<unsigned char> buffer( imageData.first, imageData.first + imageData.second );
         * cv::Mat image = imdecode(buffer, cv::IMREAD_ANYCOLOR);
         * \endcode
         *
         * \return std::pair<unsigned char*, int> - blob obrazu z kamery wzraz z jego długością
         *
         */
        std::pair<unsigned char*, int> getSendVideoFrame();

        /** \brief Rozpoczyna zapisywanie serii klatek ze strumienia kamery do folderu \c "video_record/"
         *
         * Kolejne klatki zapisywane są w formacie \c .jpg
         *
         * \return void
         *
         */
        void startRecording();
        /** \brief Kończy zapisywanie serii klatek
         *
         * \return void
         *
         */
        void stopRecording();

        /** \brief Włącza wyświetlanie dodatkowych informacji
         *
         * \return void
         *
         */
        void enableVerboseMode();

    private:
        char my_cameraName[255], my_cameraType[255],
             my_cameraNameForUserDisplay[255],
             my_cameraTypeForUserDisplay[255];/**< Informacje dotyczące kamery */

        bool my_verboseMode;/**< Stan opcji wyświetlania dodatkowych informacji */

        // Camera parameters
        int my_camera_minPan, my_camera_maxPan; /**< zakres ruchu kamery w poziomie */
        int my_camera_minTilt, my_camera_maxTilt;/**< zakres ruchu kamery w pionie */
        int my_camera_minZoom, my_camera_maxZoom;/**< zakres przybliżania optycznego kamery */
        bool my_camera_isZoomAvailable;/**< opisuje dostępność opcji przybliżania optycznego kamery */

        // Current camera position
        int my_camera_pan, my_camera_tilt,
            my_camera_zoom;/**< aktualne współrzędne opisujące stan konfiguracji kamery */

        // Send video
        unsigned char my_lastSnap[38400];/**< ostatnia pobrana klatka ze strumienia obrazu z kamery */
        int my_lastSnapSize, my_sendVideoDelay; /**< dane dotyczący strumienia obrazu z kamery */
        bool my_video_mutexOn;/**< mutex blokujący dostęp do \c my_lastSnap[] */

        // Frame recording variables
        bool my_recordToFolder;/**< stan opcji nagrywania strumienia obrazu z kamery do plików \c .jpg */
        int my_frame_number, my_filename_length;/**< dane dotyczące nagrywania strumienia */
        std::string my_file_extension;/**< rozszerzenie plików, do których zapisywane są klatki ze strumienia */

        // Camera key steering
        bool my_cameraSteeringActiveStatus;/**< stan opcji sterowania kamerą klawiaturą */

    private:
        ArClientBase* my_client;/**< wskaźnik do klienta Aria */
        keyHandlerMaster* my_keyHandler;/**< wskaźnik do obiektu klasu \c robotManager::keyHandlerMaster */

        void recordFrame(unsigned char* image, int length_of_image );/**< \brief Zapisuje przekazaną klatkę do kolejnego pliku \c .jpg */

        // CALLBACKS FUNCTIONS
        void handle_getCameraList( ArNetPacket* packet ); /**< callback polecenia \c getCameraList */
        void handle_snapshot( ArNetPacket* packet);/**< callback polecenia \c snapshot */
        void handle_getCameraInfoCamera_1( ArNetPacket* packet);/**< callback polecenia \c getCameraInfoCamera */
        void handle_getCameraDataCamera_1( ArNetPacket* packet);/**< callback polecenia \c getCameraDataCamera */
        // Key handling (S)
        void handle_key_w(void);/**< odchylenie kamery*/
        void handle_key_s(void);/**< pochylenie kamery */
        void handle_key_a(void);/**< obrót kamery w lewo */
        void handle_key_d(void);/**< obrót kamery w prawo */
        void handle_key_r(void);/**< przybliżenie optyczne */
        void handle_key_f(void);/**< oddalenie optyczne */
        // Key handling (E)

        // CALBACKS FUNCTORS
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_getCameraList;/**< functor dla polecenie \c getCameraList */
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_snapshot;/**< functor dla polecenie \c snapshot */
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_handle_getCameraInfoCamera_1;/**< functor dla polecenie \c getCameraInfoCamera */
        ArFunctor1C<cameraManager, ArNetPacket*> my_functor_hanlde_getCameraDataCamera_1;/**< functor dla polecenie \c getCameraDataCamera */
        // Key handling (S)
        ArFunctorC<cameraManager> my_functor_handle_key_w;/**< functor do obsługi klawisza \c W */
        ArFunctorC<cameraManager> my_functor_handle_key_s;/**< functor do obsługi klawisza \c S */
        ArFunctorC<cameraManager> my_functor_handle_key_a;/**< functor do obsługi klawisza \c A */
        ArFunctorC<cameraManager> my_functor_handle_key_d;/**< functor do obsługi klawisza \c D */
        ArFunctorC<cameraManager> my_functor_handle_key_r;/**< functor do obsługi klawisza \c E */
        ArFunctorC<cameraManager> my_functor_handle_key_f;/**< functor do obsługi klawisza \c F */
        // Key handling (E)
    };

    /** \brief Sterowanie robotem
     *
     * Klasa udostępnia metody umożliwiające sterowanie robotem (poruszanie się),
     * również za pomocą klawiatury:
     * \li strzałka w górę - jazda do przodu
     * \li strzałka w dół - jazda do tyłu
     * \li strzałka w lewo - obrót w lewo
     * \li strzałka w prawo - obrót w prawo
     * \li spacja - zatrzymanie robota
     */
    class steeringManager
    {
    public:
        /** \brief Konstruktor klasy \c robotManager::steeringManager
         *
         * \param _client ArClientBase* - klient Aria
         * \param _keyHandler keyHandlerMaster* - obiekt klasy \c robotManager::keyHandlerMaster
         * \param _activateKeySteering - domyślny stan opcji sterowania za pomocą klawiatury
         *
         */
        steeringManager( ArClientBase *_client, keyHandlerMaster *_keyHandler,
                         bool _activateKeySteering = true);

        /** \brief Przejechanie robotem w przód / tył o zadaną wartość
         *
         * \param distance_mm double - wartość o jaką robot ma się przesunąć
         * \return void
         *
         */
        void moveDistance( double distance_mm );
        /** \brief Obrót robotem o zadany kąt
         *
         * \param angle_deg double - kąt w stopniach o jaki robot ma się obrócić
         * \return void
         *
         */
        void turnByAngle( double angle_deg );
        /** \brief Obrót robote na azymut
         *
         * \param angle_deg double - kąt w stopniach, do którego robot ma się obrócić
         * \return void
         *
         */
        void turnToHeading( double angle_deg );

        /** \brief Aktywacja sterowania prędkościowego za pomocą klawiatury
         *
         * \return void
         *
         */
        void enableVelocitySteering();
        /** \brief Aktywacja sterowania położeniowego za pomocą klawiatury
         *
         * \return void
         *
         */
        void enableDistSteering();

        /** \brief Włącza wyświetlanie dodatkowych informacji
         *
         * \return void
         *
         */
        void enableVerboseMode();

    private:
        bool my_verboseMode, my_keySteeringActiveStatus,
             my_isRunningByKeys, my_isVelocitySteering;

        const int VEL_PERC;

        double my_velThrottle, my_rotThrottle;

    protected:
        ArClientBase* my_client;
        keyHandlerMaster* my_keyHandler;
        ArClientRatioDrive my_clientRatioDrive;

        void activateKeySteering(void);
        void deactivateKeySteering(void);

        void handle_jogModeRequests( int type, double value );

        // CALLBACKS FUNCTIONS
        void handle_key_up(void);
        void handle_key_down(void);
        void handle_key_left(void);
        void handle_key_right(void);
        void handle_key_space(void);
        void callback_keySteeringCallback(void);

        // CALLBACKS FUNCTORS
        ArFunctorC<steeringManager> my_functor_handle_key_up;
        ArFunctorC<steeringManager> my_functor_handle_key_down;
        ArFunctorC<steeringManager> my_functor_handle_key_left;
        ArFunctorC<steeringManager> my_functor_handle_key_right;
        ArFunctorC<steeringManager> my_functor_handle_key_space;
        ArFunctorC<steeringManager> my_functor_callback_keySteeringCallback;
    };

private:
    ArArgumentParser parser;
    ArClientBase client;
    ArClientSimpleConnector clientConnector;

    bool my_isClienRunning;

public:
    requestsHandler* requests; /**< Ale jajaja */
    steeringManager* steering; /**< No nie moge xD */
    cameraManager* camera;
    keyHandlerMaster* keyHandler;
};

#endif // ROBOTMANAGER_H_INCLUDED
