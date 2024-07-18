#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define REAR_ENA 25
#define REAR_R_IN1 26
#define REAR_R_IN2 27
#define REAR_L_IN3 14
#define REAR_L_IN4 12
#define REAR_ENB 13

#define FRONT_ENA 15
#define FRONT_R_IN1 2
#define FRONT_R_IN2 4
#define FRONT_L_IN3 16
#define FRONT_L_IN4 17
#define FRONT_ENB 5

#define HALF_PI 1.57079632679

typedef struct remoteMsg {
    unsigned int count;
    unsigned char r;
    float theta;
    bool btn;
} remoteMsg;

remoteMsg controlSignal;

unsigned char wheelPins[8] = {REAR_R_IN1, REAR_R_IN2, REAR_L_IN3, REAR_L_IN4, FRONT_R_IN1, FRONT_R_IN2, FRONT_L_IN3, FRONT_L_IN4}; 
unsigned char wheelEnables[4] = {REAR_ENA, REAR_ENB, FRONT_ENA, FRONT_ENB};

/**
 * @brief  Sets one of the wheels to a desired speed.
 * 
 * @param  whl:  Wheel to control. Range: [0, 3]. (0)RearR, (1)RearL, (2)FrontR, (3)FrontL
 * @param  spd:  Ratio of maximum motor output in either direction. Range: [-255, 255].
 */
void setWheel(unsigned char whl, int spd)
{
    bool fwd = spd > 0;
    digitalWrite(wheelPins[2 * whl], fwd ? HIGH : LOW);
    digitalWrite(wheelPins[2 *  whl + 1], fwd ? LOW : HIGH);
    analogWrite(wheelEnables[whl], abs(spd));
}

void brake()
{
    for (unsigned char in : wheelPins)
    {
        digitalWrite(in, LOW);
    }
    for (unsigned char en : wheelEnables)
    {
        analogWrite(en, 255);
    }
}

float leftPwmCoeff(float angle)
{
    if(angle <= -HALF_PI) return -1;
    else if(angle <= 0) return cos(2 * angle);
    else if(angle <= HALF_PI) return 1;
    else return sin(2 * angle - HALF_PI);
}

float rightPwmCoeff(float angle)
{
    if(angle <= -HALF_PI) return cos(2 * angle);
    else if(angle <= 0) return -1;
    else if(angle <= HALF_PI) return sin(2 * angle - HALF_PI);
    else return 1;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) 
{
    memcpy(&controlSignal, incomingData, sizeof(controlSignal));
    Serial.print(controlSignal.count); Serial.print(",");
    Serial.print(controlSignal.btn); Serial.print(",");
    if(controlSignal.r < 30)
    {
        controlSignal.r = 0;
    }
    if(controlSignal.btn)
    {
        brake();
        Serial.println("0,0");
    }
    else
    {
        int leftPWM = ceil(controlSignal.r * leftPwmCoeff(controlSignal.theta));
        int rightPWM = ceil(controlSignal.r * rightPwmCoeff(controlSignal.theta));
        Serial.print(leftPWM); Serial.print(","); Serial.println(rightPWM);
        setWheel(0, rightPWM); setWheel(2, rightPWM);
        setWheel(1, leftPWM); setWheel(3, leftPWM);
    }
}

void setup() 
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;
    pinMode(REAR_ENA, OUTPUT);
    pinMode(REAR_R_IN1, OUTPUT);
    pinMode(REAR_R_IN2, OUTPUT);
    pinMode(REAR_L_IN3, OUTPUT);
    pinMode(REAR_L_IN4, OUTPUT);
    pinMode(REAR_ENB, OUTPUT);

    pinMode(FRONT_ENA, OUTPUT);
    pinMode(FRONT_R_IN1, OUTPUT);
    pinMode(FRONT_R_IN2, OUTPUT);
    pinMode(FRONT_L_IN3, OUTPUT);
    pinMode(FRONT_L_IN4, OUTPUT);
    pinMode(FRONT_ENB, OUTPUT);

    digitalWrite(REAR_R_IN1, LOW);
    digitalWrite(REAR_R_IN2, LOW);
    digitalWrite(REAR_L_IN3, LOW);
    digitalWrite(REAR_L_IN4, LOW);
    analogWrite(REAR_ENA, 0);
    analogWrite(REAR_ENB, 0);

    digitalWrite(FRONT_R_IN1, LOW);
    digitalWrite(FRONT_R_IN2, LOW);
    digitalWrite(FRONT_L_IN3, LOW);
    digitalWrite(FRONT_L_IN4, LOW);
    analogWrite(FRONT_ENA, 0);
    analogWrite(FRONT_ENB, 0);

    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    
}