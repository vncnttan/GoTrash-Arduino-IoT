#include <gotrashble.h>
#include <map>

// Variable List
BLEService bleService("180D");
BLECharacteristic bleCharacteristics("2A37", BLERead | BLEWrite | BLEWriteWithoutResponse, 20);
std::map<int, unsigned long> userMap;


const unsigned long TIME_LIMIT = 0.5 * 60 * 1000; // 30 Seconds

int getCurrentUser(){
  int currentUser = -1;
  unsigned long latestTimestamp = 0;

  for (const auto& user : userMap) {
    if (user.second > latestTimestamp) {
      latestTimestamp = user.second;
      currentUser = user.first;
    }
  }

  return currentUser;  
}

void noticeUser(int trashID, int userID){
  if (trashID != -1) {
    String trashIdStr = String(trashID);
    String userIdStr = String(userID);
    String combined = userIdStr + "," + trashIdStr;
    bleCharacteristics.writeValue(combined.c_str());
    Serial.println("Sent User ID: " + combined + " to BLE characteristic with length: " + String(combined.length()) + " the C Str is " + String(combined.c_str()));
  } else {
    Serial.println("No users found to send.");
  }
}

void addUsers(int id) {
  unsigned long currentTime = millis();

  // Check if the user ID exists in the userSet
  auto it = userMap.find(id);
  if (it != userMap.end()) {
    // User exists, update timestamp in userQueue
    it->second = currentTime;
    Serial.println("Updated User: " + String(id));
  } else {
    // User does not exist, add new user
    User newUser;
    newUser.id = id;
    newUser.timestamp = currentTime;

    userMap.insert(std::make_pair(id, currentTime));
    Serial.println("Added User: " + String(id));
  }
}

void checkExpiredUser() {
   unsigned long currentTime = millis();

  for (auto it = userMap.begin(); it != userMap.end(); ) {
    // Check if the user is expired
    if (currentTime - it->second > TIME_LIMIT) {
      Serial.println("Removed User: " + String(it->first));
      // Erase expired user and advance iterator
      it = userMap.erase(it);
    } else {
      ++it;
    }
  }
}

void BLETask(void *pvParameters){
  for(;;){
    loopBLE();
  }
}

void setupBLE(){
  BLE.begin();
  BLE.setLocalName("GoTrash");
  BLE.setDeviceName("GoTrash");
  BLE.setAdvertisedService(bleService);
  
  bleCharacteristics.canWrite();
  bleCharacteristics.canRead();
  bleCharacteristics.writeValue("");

  bleService.addCharacteristic(bleCharacteristics);
  BLE.addService(bleService);
  BLE.advertise();

  xTaskCreate(BLETask, "BLETask", 10000, NULL, 1, NULL);
  Serial.println("GoTrash BLE Started!");
}

void loopBLE(){
  BLEDevice central = BLE.central();
  checkExpiredUser();
  if (central && central.connected() &&  bleCharacteristics.written()) {
    const uint8_t* value = bleCharacteristics.value();
    int length = bleCharacteristics.valueLength();

    String receivedValue = "";
    for (int i = 0; i < length; i++) {
      receivedValue += (char)value[i];
    }
    addUsers(receivedValue.toInt());    
  }
}