#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "esp_system.h"

hw_timer_t *timer = NULL;
void IRAM_ATTR resetModule(){
    Serial.println("reboot\n");
    esp_restart();
}

#define SSID        "POCO X6 5G"
#define PASSWORD    "zkqe8842"

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* apiEndpoint = "http://194.238.19.17:5002/upload";

// Your Sensor Pins
const int trigPin = 13;
const int echoPin = 15;
boolean startTimer = false;
unsigned long time_now = 0;
int pinSensor = 1, Cam_capture = 0 ,time_capture=0;
long duration;
int distance;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Picture");
  Serial.flush();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  WiFi.begin(SSID, PASSWORD);

  Serial.printf("WiFi connecting to %s\n",  SSID);
  while(WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(400); }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());  
  
  // Initialize timer 0 with a prescaler of 80 (1MHz frequency)
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &resetModule);
  timerAlarm(timer, 1000000, true, 0);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    // FRAMESIZE_ +
    //QQVGA/160x120//QQVGA2/128x160//QCIF/176x144//HQVGA/240x176
    //QVGA/320x240//CIF/400x296//VGA/640x480//SVGA/800x600//XGA/1024x768
    //SXGA/1280x1024//UXGA/1600x1200//QXGA/2048*1536
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  Serial.println("I'm here 0");
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  timerWrite(timer, 0); //reset timer (feed watchdog)

  if(Cam_capture == 1){
    Camera_capture();
  }
  if(pinSensor == 1){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration*0.034/2;
          
    if(distance > 2 and distance <= 30 && startTimer != true){
      Camera_capture();
      startTimer = true;
    } else if(distance > 300){
      startTimer = false;
      time_capture=0;
    }
    if(distance > 2 and distance <= 30){
      if(++time_capture > 4){
        time_capture=0;
        Camera_capture();
        Serial.println("Over Time");
      }
    }
  }
  delay(500);
}

void Camera_capture() {
  Serial.println("Capturing with camera");
  camera_fb_t * fb = NULL;
  delay(200); 

  // Activate flash
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);  
  delay(1000); 

  // Take Picture with Camera
  fb = esp_camera_fb_get(); 
  if(!fb) {
    Serial.println("Camera capture failed");
    digitalWrite(4, LOW);
    return;
  }
  delay(2000);
  digitalWrite(4, LOW);

  Send_request(fb->buf,fb->len, fb);
  
  esp_camera_fb_return(fb);
  Serial.println("OK"); 
}

void Send_request(uint8_t *image_data, size_t image_size, camera_fb_t * fb){
  Serial.println("Sending Request...");
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(apiEndpoint);
    http.addHeader("Content-Type", "image/jpeg");
    
    int httpResponseCode = http.POST(fb->buf, fb->len);
    
    esp_camera_fb_return(fb); 
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    
    http.end();
  }

  delay(1000);
}



