using namespace std;

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

unsigned long lastBlink = 0;
unsigned long lastSend = 0;
unsigned long lastReconnectAttempt = 0;
bool ready = true;

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

const char *MQTTserver =  "192.168.1.85";

string topic, errorTopic, debugTopic;
char chipID[9];

WiFiClient wclient;
PubSubClient mqtt_broker(wclient);

void setup(void) {
  pinMode(flashPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED OFF

  sprintf (chipID, "%06x", ESP.getChipId());

  topic = "sensor/" + (string)chipID;
  errorTopic = topic + "/error";
  debugTopic = topic + "/debug";

  Serial.begin(115200);

  WiFiManager wifiManager;
  wifiManager.autoConnect();

  mqtt_broker.setServer(MQTTserver, 1883);

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

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }
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
