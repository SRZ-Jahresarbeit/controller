//#include <Arduino.h>
class Arduino;
#include <WiFi.h>

class Wifi
{
private:
    char *ssid;
    char *password;
    char *hostname;

    uint8_t tries;
    uint8_t connectTimeout; // seconds

public:
    Wifi(char ssid[], char password[], char hostname[] = "Arduino T-Systems MMS", uint8_t tries = 4, uint8_t connectTimeout = 15);

    bool connect();

    bool update();

    wl_status_t status();
};