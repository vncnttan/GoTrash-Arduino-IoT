#include <Servo.h>

const int servoCount = 3;// how many servo 
static const int servosPins[servoCount] = {12, 14, 27}; // define pins here

Servo_ESP32 servos[servoCount];//do not change

void setServos(int degrees) {
    for(int i = 0; i < servoCount; ++i) {
        servos[i].write((degrees + (35 * i)) % 180);
    }
}

void setup() {
  Serial.begin(115200);
	Serial.print("Robojax ESP32 Multiple servo");

    for(int i = 0; i < servoCount; ++i) {
        if(!servos[i].attach(servosPins[i])) {
            Serial.print("Servo ");
            Serial.print(i);
            Serial.println("attach error");
        }
    }
}

void loop() {
    for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        setServos(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }


    for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        setServos(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }
}
