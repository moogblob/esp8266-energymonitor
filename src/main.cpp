using namespace std;
#include <FS.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

unsigned long lastBlink = 0;
unsigned long lastSend = 0;
unsigned long lastReconnectAttempt = 0;
bool ready = true;

bool shouldSaveConfig = false;

unsigned long lastGatherRealtime = 0;
unsigned long lastGatherMinute = 0;
unsigned long lastGatherHour = 0;
unsigned long lastGatherDay = 0;

unsigned int impulseCounterMinute = 0;
unsigned int impulseCounterHour = 0;
unsigned int impulseCounterDay = 0;

string payloadDay, payloadHour, payloadMinute;

int pulseDistance[3] = {1,1,1};
int pulseDistanceIndex = 0;

float kwMinute, kwHour, kwDay = 0.00f;
float kwhMinute, kwhHour, kwhDay = 0.00f;

int minSampleIntervalMS = 250; // At a rate of 250ms between impulses, the maxiumum power it can measure is (3600/250) = 14.4 kW. Lower value if needed.
#ifdef FLASHPIN
  int flashPin = FLASHPIN;
#else
  int flashPin = D5;
#endif
int feedbackBlinkTimer = 30;

char mqtt_server[40] = "192.168.1.85";
char mqtt_port[6] = "1883";

string topic, errorTopic, debugTopic;
char chipID[9];

WiFiClient wclient;
PubSubClient mqtt_broker(wclient);

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup(void) {

  //SPIFFS.format();

  pinMode(flashPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED OFF

  sprintf (chipID, "%06x", ESP.getChipId());

  topic = "sensor/" + (string)chipID;
  errorTopic = topic + "/error";
  debugTopic = topic + "/debug";

  Serial.begin(115200);

  //read configuration from FS json
  Serial.println("mounting FS...");
  yield();

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  WiFiManager wifiManager;

  WiFiManagerParameter custom_mqtt_server("mqtt-server", "mqtt server", mqtt_server, 40);
  wifiManager.addParameter(&custom_mqtt_server);

  WiFiManagerParameter custom_mqtt_port("mqtt-port", "mqtt port", mqtt_port, 6);
  wifiManager.addParameter(&custom_mqtt_port);

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  //wifiManager.resetSettings();

  if (!wifiManager.autoConnect("Energymonitor")) {
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  // Read values
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());

  Serial.printf("ADRSES: '%s'", mqtt_server);
  Serial.printf("PORT: '%s'", mqtt_port);

  // Connect MQTT broker TODO: Add credentials
  mqtt_broker.setServer(mqtt_server, atoi(mqtt_port));

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  ArduinoOTA.onStart([]() {
    Serial.println("Start start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

bool reconnect() {
  String clientId = "E-";
  clientId += chipID;
  if (mqtt_broker.connect(clientId.c_str())) {
    Serial.println("MQTT connected.");
  }
  return mqtt_broker.connected();
}

float getAverageDuration() {
  float totalDuration = 0;
  for (int i=0;i<3;i++) {
    totalDuration += pulseDistance[i];
  }
  return totalDuration/3.00f;
}

bool updateStatistics(unsigned long* lastGather, float interval, unsigned int* impulseCounter, float* kwval, float* kwhval) {
  // current power use = seconds per hour / (seconds between flashes * impressions per kWh)
  // For a meter with 1000 imp/kWh and a time of 3 seconds between flashes, the current power use is 1.2 kW.
  // 3600 / (3 * 1000) = 1.2 kW
  unsigned long now = millis();
  if ((now - *lastGather) > (interval*1000.00f)) {
      *lastGather = now;

      if (*impulseCounter>0) {
        *kwval = 3600.00f / ((interval / *impulseCounter) * 1000.00f);
        *kwhval = *impulseCounter / 1000.00f;
      } else {
        *kwval = 0.00f;
        *kwhval = 0.00f;
      }
      *impulseCounter = 0;
      return true;
  }
  return false;
}

bool getImpulse() {
  unsigned long now = millis();

  if (digitalRead(flashPin) == LOW && (now - lastBlink) > minSampleIntervalMS && ready==true) {
    pulseDistance[pulseDistanceIndex] = now - lastBlink;
    if(++pulseDistanceIndex>=3) {
      pulseDistanceIndex = 0;
    }
    ready = false;
    lastBlink = now;
    impulseCounterDay++;
    impulseCounterHour++;
    impulseCounterMinute++;

    return true;
  }
  if (digitalRead(flashPin) == HIGH) {
    ready = true;
  }
  return false;
}

void userFeedback() {
  if (feedbackBlinkTimer>0) {
    digitalWrite(LED_BUILTIN, LOW); // LED ON
    feedbackBlinkTimer--;
  } else {
    digitalWrite(LED_BUILTIN, HIGH); // LED OFF
  }
}

void sendWithMQTT() {
  // Send to broker every 2 sec
  unsigned long now = millis();

  if (now - lastSend > 2*1000) {
    lastSend = now;
    float realtimePower;
    // Is it recently initialized?
    if (pulseDistance[0] == 1 && pulseDistance[1] == 1 && pulseDistance[2] == 1) {
      realtimePower = 0.00f;
    } else {
      realtimePower = 3600.00f / getAverageDuration();
    }

    // KW
    char kwNowString[6];
    dtostrf(realtimePower, 5, 2, kwNowString);

    char kwMinuteString[6];
    dtostrf(kwMinute, 5, 2, kwMinuteString);

    char kwHourString[6];
    dtostrf(kwHour, 5, 2, kwHourString);

    char kwDayString[6];
    dtostrf(kwDay, 5, 2, kwDayString);

    // KWH
    char kwhMinuteString[6];
    dtostrf(kwhMinute, 5, 2, kwhMinuteString);

    char kwhHourString[6];
    dtostrf(kwhHour, 5, 2, kwhHourString);

    char kwhDayString[6];
    dtostrf(kwhDay, 5, 2, kwhDayString);

    unsigned long uptimeString = now/1000;

    char payload[256];
    char payload_kw[128];
    char payload_kwh[128];

    String resetReasonString = ESP.getResetReason();

    char resetReason[32];
    strcpy(resetReason, resetReasonString.c_str());

    sprintf(payload_kw, "\"kw\":{\"now\":%s,\"minute\":%s,\"hour\":%s,\"day\":%s}", kwNowString, kwMinuteString, kwHourString, kwDayString);
    sprintf(payload_kwh, "\"kwh\":{\"minute\":%s,\"hour\":%s,\"day\":%s}", kwhMinuteString, kwhHourString, kwhDayString);
    sprintf(payload, "{%s, %s, \"uptime\":%lu, \"free_heap\":%u, \"reset_reason\":\"%s\"}", payload_kw, payload_kwh, uptimeString, ESP.getFreeHeap(), resetReason);

    mqtt_broker.publish(topic.c_str(), payload, true);

  }
}

void loop() {
  ArduinoOTA.handle();

  if (getImpulse()) {
    feedbackBlinkTimer = 100;
  }

  updateStatistics(&lastGatherMinute, 60.0f, &impulseCounterMinute, &kwMinute, &kwhMinute);
  updateStatistics(&lastGatherHour, 3600.0f, &impulseCounterHour, &kwHour, &kwhHour);
  updateStatistics(&lastGatherDay, 86400.0f, &impulseCounterDay, &kwDay, &kwhDay);

  userFeedback();
  yield();

  if (!mqtt_broker.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    yield();
    mqtt_broker.loop();
    sendWithMQTT();
  }

}
