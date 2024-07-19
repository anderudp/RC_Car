#include <Arduino.h>
#include "WiFi.h"
#include <esp_now.h>

#define VRx 33
#define VRy 32
#define SW 13
#define LED 2

uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x03, 0x03, 0x90};

unsigned int counter = 0;

typedef struct remoteMsg {
    unsigned int count;
    unsigned char r;
    float theta;
    bool btn;
} remoteMsg;

remoteMsg controlSignal;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) 
{
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        Serial.println(",0");
        analogWrite(LED, 127);
        delay(10);
        analogWrite(LED, 0);
    }
    else
    {
        Serial.println(",1");
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(VRx, INPUT);
    pinMode(VRy, INPUT);
    pinMode(SW, INPUT_PULLUP);
    pinMode(LED, OUTPUT);
    adcAttachPin(VRx);
    adcAttachPin(VRy);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) return;

    counter = 0;
}

void loop()
{
    controlSignal.count = counter;

    bool sw = 1 - digitalRead(SW);
    controlSignal.btn = sw;

    int x = map(analogRead(VRx), 0, 4095, -255, 255);
    int y = map(analogRead(VRy), 0, 4095, -255, 255);
    // Chebyshev distance
    unsigned char r = max(abs(x), abs(y));
    controlSignal.r = r;

    float theta = atan2(x, y);
    controlSignal.theta = theta;

    Serial.print(counter++); Serial.print(",");
    Serial.print(sw ? 1 : 0); Serial.print(",");
    Serial.print(r); Serial.print(",");
    Serial.print(theta);

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &controlSignal, sizeof(controlSignal));
    delay(100);
}