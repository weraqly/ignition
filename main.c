#include <Arduino.h>
#include <Servo.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


const char* AP_SSID = "LAB5_ESP8266";
const char* AP_PASS = "lab5pass123"; 


const char* MQTT_HOST = "192.168.4.2";
const uint16_t MQTT_PORT = 1884;

const char* MQTT_USER = "";
const char* MQTT_PASS = "";


const char* DEVICE_ID = "lab5-elevator-01";
const char* LAB_ID    = "lab5";

const char* MQTT_TOPIC_DATA   = "lab5/data";
const char* MQTT_TOPIC_STATUS = "lab5/status";

const unsigned long MQTT_PUB_MS = 2000;



#define upBut 13
#define downBut 12
#define servoPin 14
#define dht11 0


const int MAX_FLOORS = 9;
const int MIN_FLOORS = 1;

const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE   = 90;

const unsigned long FLOOR_TRAVEL_MS = 1800;
const unsigned long DOOR_HOLD_MS    = 2500;
const unsigned long DEBOUNCE_MS     = 40;
const unsigned long DHT_READ_MS     = 2000;


struct ButtonDebounce {
  bool lastReading = HIGH;
  bool stableState = HIGH;
  unsigned long lastChangeMs = 0;
};
enum class ElevatorState : uint8_t { Idle, Moving, DoorOpen };


bool wasPressed(uint8_t pin, ButtonDebounce &b);
const char* stateToStr(ElevatorState s);

DHT dht(dht11, DHT11);
Servo doorServo;
bool doorIsOpen = false;


WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

ButtonDebounce btnUp, btnDown;

ElevatorState state = ElevatorState::Idle;
int currentFloor = 1;
int targetFloor  = 1;

unsigned long nextMoveMs   = 0;
unsigned long doorCloseMs  = 0;

float lastTemp = NAN;
float lastHum  = NAN;
unsigned long nextDhtRead = 0;
unsigned long nextMqttPublish = 0;

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  
  Serial.println();
  Serial.println("AP started!");
  Serial.print("SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("Connect your laptop to this Wi-Fi. Laptop IP will be ~192.168.4.2");
}

void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  while (!mqtt.connected()) {
    Serial.print("MQTT connecting... ");

    bool ok;
    if (strlen(MQTT_USER) > 0) {
      ok = mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS, MQTT_TOPIC_STATUS, 0, true, "offline");
    } else {
      ok = mqtt.connect(DEVICE_ID, MQTT_TOPIC_STATUS, 0, true, "offline");
    }

    if (ok) {
      Serial.println("OK");
      mqtt.publish(MQTT_TOPIC_STATUS, "online", true);
    } else {
      Serial.print("FAIL rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

bool wasPressed(uint8_t pin, ButtonDebounce &b) {
  bool reading = digitalRead(pin);
  unsigned long now = millis();

  if (reading != b.lastReading) {
    b.lastChangeMs = now;
    b.lastReading = reading;
  }

  if ((now - b.lastChangeMs) > DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) return true;
  }
  return false;
}

const char* stateToStr(ElevatorState s) {
  switch (s) {
    case ElevatorState::Idle:     return "Idle";
    case ElevatorState::Moving:   return "Moving";
    case ElevatorState::DoorOpen: return "DoorOpen";
  }
  return "Unknown";
}

void openDoor() { doorServo.write(SERVO_OPEN_ANGLE); doorIsOpen = true; }
void closeDoor(){ doorServo.write(SERVO_CLOSED_ANGLE); doorIsOpen = false; }

void publishJson() {
  if (!mqtt.connected()) return;

  char payload[220];
  char tempBuf[16], humBuf[16];

  if (isnan(lastTemp)) strcpy(tempBuf, "null"); else dtostrf(lastTemp, 0, 1, tempBuf);
  if (isnan(lastHum))  strcpy(humBuf,  "null"); else dtostrf(lastHum,  0, 0, humBuf);

  snprintf(payload, sizeof(payload),
    "{\"lab\":\"%s\",\"device\":\"%s\",\"floor\":%d,\"target\":%d,"
    "\"doorOpen\":%s,\"temperature\":%s,\"humidity\":%s,\"rssi\":%d,\"state\":\"%s\"}",
    LAB_ID, DEVICE_ID,
    currentFloor, targetFloor,
    doorIsOpen ? "true" : "false",
    tempBuf, humBuf,
    WiFi.RSSI(),
    stateToStr(state)
  );

  mqtt.publish(MQTT_TOPIC_DATA, payload, false);
  Serial.println(payload);
}

void setup() {
  Serial.begin(9600);

  pinMode(upBut, INPUT_PULLUP);
  pinMode(downBut, INPUT_PULLUP);

  dht.begin();

  doorServo.attach(servoPin);
  closeDoor();

  currentFloor = MIN_FLOORS;
  targetFloor  = currentFloor;

  startAP();

  connectMQTT();

  nextDhtRead = millis();
  nextMqttPublish = millis();
  publishJson();
}

void loop() {
  unsigned long now = millis();

  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  if (now >= nextDhtRead) {
    lastTemp = dht.readTemperature();
    lastHum  = dht.readHumidity();
    nextDhtRead = now + DHT_READ_MS;
  }

  if (wasPressed(upBut, btnUp) && targetFloor < MAX_FLOORS) targetFloor++;
  if (wasPressed(downBut, btnDown) && targetFloor > MIN_FLOORS) targetFloor--;

  switch (state) {
    case ElevatorState::Idle:
      if (targetFloor != currentFloor) {
        closeDoor();
        state = ElevatorState::Moving;
        nextMoveMs = now + FLOOR_TRAVEL_MS;
      }
      break;

    case ElevatorState::Moving:
      if (now >= nextMoveMs) {
        int dir = (targetFloor > currentFloor) ? 1 : -1;
        currentFloor += dir;

        if (currentFloor == targetFloor) {
          openDoor();
          state = ElevatorState::DoorOpen;
          doorCloseMs = now + DOOR_HOLD_MS;
        } else {
          nextMoveMs = now + FLOOR_TRAVEL_MS;
        }
      }
      break;

    case ElevatorState::DoorOpen:
      if (now >= doorCloseMs) {
        closeDoor();
        state = ElevatorState::Idle;
      }
      break;
  }

  if (now >= nextMqttPublish) {
    nextMqttPublish = now + MQTT_PUB_MS;
    publishJson();
  }

  delay(10);
}
