#ifndef GOTRASHBLE_H
#define GOTRASHBLE_H

#include <map> 
#include <ArduinoBLE.h>
#include <Arduino.h>

struct User {
  int id;
  unsigned long timestamp;
};

extern std::map<int, unsigned long> userMap;

int getCurrentUser();
void noticeUser(int trashID, int userID);
void setupBLE();
void loopBLE();

#endif // GOTRASHBLE_H