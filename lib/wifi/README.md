```c++
#include <Arduino.h>
#include <wifi.h>

#define SSID "ssid"
#define SSID_PASS "password"

Wifi wifi(SSID, SSID_PASS);

void setup()
{
    Serial.begin(115200);
    sleep(5); // wait for serial

    if(!wifi.connect()) // optional, see update
    {
        // connect error
    }
    else
    {
        // code witch is depending of wifi
    }
}

void loop()
{
    if (!wifi.update()) // reconnect or connect if not connected
    {
        return;
    }

    // code witch is depending on wifi
}
```
