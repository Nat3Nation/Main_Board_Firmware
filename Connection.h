#include <WebSocketsClient.h>
#include <string.h>
#include "PCA9671BS.h"

#ifndef Connection_h
#define Connection_h

class Connection
{
  public:
    Connection(String serverName);
    /* initialize the wifi connection by indicating with the ssid and password */
    void initWiFi();
    /* send a HTTP post */
    void HTTP_post(String const & message);
    
    void loop();

    void send(const char * data);

    void initBackend();

    static void onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length);

    String get_wifi_ssid();

    String get_wifi_pass();

    void ping_LoRa_Backend();


  private:
    unsigned long lastTime_;
    unsigned long timerDelay_;
    String serverName_;
    WebSocketsClient webSocket_;
    WiFiClient client_;
};

#endif