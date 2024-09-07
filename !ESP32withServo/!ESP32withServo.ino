#include <ESP32Servo.h>
#include <HardwareSerial.h>
// #include <gotrashble.h>

HardwareSerial SerialPort(2); 

Servo servo1;
Servo servo2; 
Servo servo3;

int minUs = 550;
int maxUs = 2400;
int servo1Pin = 12;
int servo2Pin = 14; // centerServo
int servo3Pin = 27;

int pos = 0;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Serial.begin(115200);

  // setupBLE();
  servo1.setPeriodHertz(50);  // Standard 50hz servo
  servo2.setPeriodHertz(50);  // Standard 50hz servo
  servo3.setPeriodHertz(50);  // Standard 50hz servo

  resetServo(servo1, servo1Pin, false);
  resetServo(servo2, servo2Pin, true); 
  resetServo(servo3, servo3Pin, false);

  // randomSeed(100);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17);
}

void loop() {
  // loopBLE();
  if (SerialPort.available()) {
    String receivedData = SerialPort.readStringUntil('\n');
    Serial.println(receivedData);
    if (receivedData.startsWith("FUNC:")) {
      int trashClassification = receivedData.substring(5).toInt();

      Serial.print("Trash Classification: ");
      Serial.println(trashClassification);

      switch (trashClassification) {
        case 0:
          rotateRightServo(servo1, servo1Pin, false);
          rotateRightServo(servo2, servo2Pin, true);
          Serial.println("Waiting for the trash to fall down...");
          delay(5000);
          resetServo(servo2, servo2Pin, true);
          resetServo(servo1, servo1Pin, false);
          break;
        case 1:
          rotateLeftServo(servo1, servo1Pin, false);
          rotateRightServo(servo2, servo2Pin, true);
          Serial.println("Waiting for the trash to fall down...");
          delay(5000);
          resetServo(servo2, servo2Pin, true);
          resetServo(servo1, servo1Pin, false);
          break;
        case 2:
          rotateRightServo(servo3, servo3Pin, false);
          rotateLeftServo(servo2, servo2Pin, true);
          Serial.println("Waiting for the trash to fall down...");
          delay(5000);
          resetServo(servo2, servo2Pin, true);
          resetServo(servo3, servo3Pin, false);
          break;
        case 3:
          rotateLeftServo(servo3, servo3Pin, false);
          rotateLeftServo(servo2, servo2Pin, true);
          Serial.println("Waiting for the trash to fall down...");
          delay(5000);
          resetServo(servo2, servo2Pin, true);
          resetServo(servo3, servo3Pin, false);
          break;
      }

      servo1.detach();
      servo2.detach();
      servo3.detach();
    }
  }
}

void rotateRightServo(Servo servo, int servoPin, bool isCenterServo) {
  servo.attach(servoPin, minUs, maxUs);
  servo.write(0);
  delay(1000);
  Serial.print(servoPin);
  Serial.println(servo.read());
  servo.detach();
}

void rotateLeftServo(Servo servo, int servoPin, bool isCenterServo) {
  servo.attach(servoPin, minUs, maxUs);
  int degree = 120;
  if (isCenterServo) {
    degree =  80;
  }

  servo.write(degree);
  delay(1000);
  Serial.println(servo.read());
  servo.detach();
}

void resetServo(Servo servo, int servoPin, bool isCenterServo) {
  servo.attach(servoPin, minUs, maxUs);
  int degree = 60;
  if (isCenterServo) {
    degree = 40;
  }

  servo.write(degree);
  delay(1000);
  Serial.println(servo.read());
  servo.detach();
}