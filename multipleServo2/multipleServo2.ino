/*
 * Original sourse: https://github.com/RoboticsBrno/ESP32-Arduino-Servo-Library
 * 
 */
#include <Servo_ESP32.h>

const int servoCount = 4;// how many servo 
static const int servosPins[servoCount] = {12, 14, 27, 26}; // define pins here

Servo_ESP32 servos[servoCount];//do not change

/*
* this funciton is to move all servos to the same angle. Watch video for instruction https://youtu.be/KUqR3ZLX5Ks
*/
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
	//Robojax.com multiple Servo using ESP32
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