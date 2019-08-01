
/*
 * Internet Of Låve
 * 
 * Measure and display Temperature and Humidity (AM2302 DHT22) and Wind Speed (MPXV7002DP), 
 * and provide these values to a Munin plugin.
 * 
 * Running on a WeMos D1 WiFi board.
 */

#include <DHTesp.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include "SHT21.h"
#include <Ticker.h>

static const char* SETUP_SSID = "sensor-setup";
static const byte  EEPROM_INITIALIZED_MARKER = 0xF1; //Just a magic number

static const uint8_t DHT_TEMP_HUMIDITY_SENSOR         = 1<<0;
static const uint8_t SHT_TEMP_HUMIDITY_SENSOR         = 1<<1;
static const uint8_t PRESSURE_DIFF_SENSOR             = 1<<2;
static const uint8_t WINDSPEED_SENSOR                 = 1<<3;
static const uint8_t FAN_RELAY                        = 1<<4;
static const uint8_t THERMOSTAT_RELAY                 = 1<<5;


#define I_PRESSURE_DIFF_PIN            (A0) //Max 3.3V

#define O_SHT_SYNC_PIN                 (D3) //SCL
#define I_SHT_DATA_PIN                 (D4) //SDA
#define I_DHT_PIN                      (D2)
#define I_SETUP_MODE_PIN               (D9) //built-in LED
#define I_WINDSPEED_PIN                (D6)
#define O_FAN_RELAY_TRIGGER_PIN        (D7)
#define O_THERMOSTAT_RELAY_TRIGGER_PIN (D8)

#define DELAY_BETWEEN_ACTIVE_SENSORS   (25) //ms between reading different sensors
#define SENSOR_READ_DELAY_TIME         (10*1000)  //ms min. time between sensor reading cycle
#define MQTT_PUBLISH_INTERVAL          (5*1000) //ms min. time between publishing sensor values to MQTT server

static const float WINDSPEED_TRAVEL_DISTANCE = 0.2262; //meter pr rotation

#define DHT_MODEL (DHTesp::AM2302)


enum DEBUG_MODE
{
  DEBUG_NONE,
  DEBUG_SERIAL,
  DEBUG_MQTT
} debug_mode = DEBUG_SERIAL;
#define MQTT_DEBUG_TOPIC "debug"

enum State
{
  SETUP_MODE,
  READ_DHT_SENSOR,
  READ_SHT_SENSOR,
  READ_PRESSURE_DIFF_SENSOR
};


DHTesp dht;
SHT21 sht;

ESP8266WebServer server(80);

WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);
long last_mqtt_publish_time = 0L;

//For SETUP_SSID AP
DNSServer dnsServer;
static const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

Ticker ticker;

volatile State state;

#define MAX_SSID_LENGTH            (32)
#define MAX_PASSWORD_LENGTH        (64)
#define MAX_ACTIVE_SENSORS_LENGTH   (1)
#define MAX_MQTT_SERVERNAME_LENGTH (64)
#define MAX_MQTT_SENSORID_LENGTH    (8)
#define MAX_MQTT_USERNAME_LENGTH   (32)
#define MAX_MQTT_PASSWORD_LENGTH   (32)

char ssid_param[MAX_SSID_LENGTH+1];
char password_param[MAX_PASSWORD_LENGTH+1];
char mqtt_servername_param[MAX_MQTT_SERVERNAME_LENGTH+1];
char mqtt_sensorid_param[MAX_MQTT_SENSORID_LENGTH+1];
char mqtt_username_param[MAX_MQTT_USERNAME_LENGTH+1];
char mqtt_password_param[MAX_MQTT_PASSWORD_LENGTH+1];
bool mqtt_enabled;
uint8_t active_sensors_param = 0;

volatile bool should_read_dht_temp_sensor = false;
float dht_tempsensor_value, published_dht_tempsensor_value;
float dht_humiditysensor_value, published_dht_humiditysensor_value;

volatile bool should_read_sht_temp_sensor = false;
float sht_tempsensor_value, published_sht_tempsensor_value;
float sht_humiditysensor_value, published_sht_humiditysensor_value;

volatile float pressure_diff_value, published_pressure_diff_value = -999.9f;

volatile unsigned long windspeed_start_time;
volatile unsigned int windspeed_count;
float published_windspeed = -999.9f;

volatile bool debug_enabled = false; // Internal, to keep track of Serial status

//Forward define
bool connectMQTT();


void enableDebug()
{
  if (!debug_enabled)
  {
    debug_enabled = true;
    if (debug_mode==DEBUG_SERIAL)
    {
      Serial.begin(9600);
    }
  }
}

void disableDebug()
{
  if (debug_enabled)
  {
    debug_enabled = false;
    if (debug_mode==DEBUG_SERIAL)
    {
      Serial.flush();
      Serial.end();
    }
  }
}

void printDebug(const char* msg)
{
  if (debug_enabled && msg)
  {
    if (debug_mode==DEBUG_SERIAL)
    {
      Serial.println(msg);
    }
    else if (debug_mode==DEBUG_MQTT && mqtt_enabled)
    {
      if (connectMQTT())
      {
        Serial.println((String("MQTT publish returned ") + String(mqtt_client.publish((String(mqtt_sensorid_param)+F(MQTT_DEBUG_TOPIC)).c_str(), msg)?"true":"false")).c_str());
      }
      else
      {
        Serial.println("Debug failed - MQTT is not connected");
      }
    }
  }
}

void ICACHE_RAM_ATTR windspeedInterruptHandler()
{
  if (0 == windspeed_count)
  {
    windspeed_start_time = millis();
  }
  windspeed_count++;
}

float getWindspeed() // m/s
{
  if (0 == windspeed_count)
  {
    return 0.0f;
  }

  unsigned long current_time = millis();
  if (current_time < windspeed_start_time)
  {
    return 0.0f;
  }

  float windspeed = (windspeed_count*WINDSPEED_TRAVEL_DISTANCE) / (current_time-windspeed_start_time); // meter/ms
  windspeed_count = 0;
  return windspeed*1000.0f; // meter/s
}

void onTick()
{
  switch(state)
  {
    case READ_DHT_SENSOR:
      {
        if (active_sensors_param & DHT_TEMP_HUMIDITY_SENSOR)
        {
          should_read_dht_temp_sensor = true;
        }

        state = READ_SHT_SENSOR;
        ticker.attach_ms(DELAY_BETWEEN_ACTIVE_SENSORS, onTick);
        break;  
      }

    case READ_SHT_SENSOR:
      {
        if (active_sensors_param & SHT_TEMP_HUMIDITY_SENSOR)
        {
          should_read_sht_temp_sensor = true;
        }
        
        state = READ_PRESSURE_DIFF_SENSOR;
        ticker.attach_ms(DELAY_BETWEEN_ACTIVE_SENSORS, onTick);
        break;
      }
    
    case READ_PRESSURE_DIFF_SENSOR:
      {
        if (active_sensors_param & PRESSURE_DIFF_SENSOR)
        {
          pressure_diff_value = max(0, min(1023, analogRead(I_PRESSURE_DIFF_PIN))) * 5.0f / 1023.0f -2.5f;
          printDebug((String("reading pressure diff value ")+String(pressure_diff_value, 2)).c_str());
        }
        
        state = READ_DHT_SENSOR;
        ticker.attach_ms(SENSOR_READ_DELAY_TIME, onTick);
        break;
      }
    
    default: break;
  }
}

void readPersistentString(char* s, int max_length, int& adr)
{
  int i = 0;
  byte c;
  do
  {
    c = EEPROM.read(adr++);
    if (i<max_length)
    {
      s[i++] = static_cast<char>(c);
    }
  } while (c!=0);
  s[i] = 0;
}

void readPersistentByte(uint8_t& b, int& adr)
{
  b = EEPROM.read(adr++);
}

void readPersistentParams()
{
  int adr = 0;
  if (EEPROM_INITIALIZED_MARKER != EEPROM.read(adr++))
  {
    ssid_param[0] = 0;
    password_param[0] = 0;
    mqtt_servername_param[0] = 0;
    mqtt_sensorid_param[0] = 0;
    mqtt_username_param[0] = 0;
    mqtt_password_param[0] = 0;
    active_sensors_param = 0;
  }
  else
  {
    readPersistentString(ssid_param, MAX_SSID_LENGTH, adr);
    readPersistentString(password_param, MAX_PASSWORD_LENGTH, adr);
    readPersistentByte(active_sensors_param, adr);
    readPersistentString(mqtt_servername_param, MAX_MQTT_SERVERNAME_LENGTH, adr);
    readPersistentString(mqtt_sensorid_param, MAX_MQTT_SENSORID_LENGTH, adr);
    readPersistentString(mqtt_username_param, MAX_MQTT_USERNAME_LENGTH, adr);
    readPersistentString(mqtt_password_param, MAX_MQTT_PASSWORD_LENGTH, adr);
  }
}

void writePersistentString(const char* s, size_t max_length, int& adr)
{
  for (int i=0; i<min(strlen(s), max_length); i++)
  {
    EEPROM.write(adr++, s[i]);
  }
  EEPROM.write(adr++, 0);
}

void writePersistentByte(uint8_t b, int& adr)
{
  EEPROM.write(adr++, b);
}

void writePersistentParams(const char* ssid, const char* password, uint8_t active_sensors)
{
  int adr = 0;
  EEPROM.write(adr++, EEPROM_INITIALIZED_MARKER);
  writePersistentString(ssid, MAX_SSID_LENGTH, adr);
  writePersistentString(password, MAX_PASSWORD_LENGTH, adr);
  writePersistentByte(active_sensors, adr);
  writePersistentString(mqtt_servername_param, MAX_MQTT_SERVERNAME_LENGTH, adr);
  writePersistentString(mqtt_sensorid_param, MAX_MQTT_SENSORID_LENGTH, adr);
  writePersistentString(mqtt_username_param, MAX_MQTT_USERNAME_LENGTH, adr);
  writePersistentString(mqtt_password_param, MAX_MQTT_PASSWORD_LENGTH, adr);
  EEPROM.commit();
}

void activateHeater(bool activate)
{
  if (active_sensors_param & THERMOSTAT_RELAY)
  {
    digitalWrite(O_THERMOSTAT_RELAY_TRIGGER_PIN, activate ? HIGH : LOW);
  }
}

void activateFan(bool activate)
{
  if (active_sensors_param & FAN_RELAY)
  {
    digitalWrite(O_FAN_RELAY_TRIGGER_PIN, activate ? HIGH : LOW);
  }
}

void handleNotFound()
{
  server.send(404, F("text/plain"), F("Page Not Found\n"));
}

void handleSetupRoot()
{
  if (server.hasArg("ssid") || server.hasArg("password")
      || server.hasArg("mqtt_server") || server.hasArg("mqtt_id") || server.hasArg("mqtt_username") || server.hasArg("mqtt_password")
      || server.hasArg("sensor0") || server.hasArg("sensor1") || server.hasArg("sensor2") || server.hasArg("sensor3")
      || server.hasArg("sensor4") || server.hasArg("sensor5") || server.hasArg("sensor6") || server.hasArg("sensor7"))
  {
    if (server.hasArg("ssid"))
    {
      strncpy(ssid_param, server.arg("ssid").c_str(), MAX_SSID_LENGTH);
      ssid_param[MAX_SSID_LENGTH] = 0;
    }
    
    if (server.hasArg("password") && !server.arg("password").equals(F("password")))
    {
      strncpy(password_param, server.arg("password").c_str(), MAX_PASSWORD_LENGTH);
      password_param[MAX_PASSWORD_LENGTH] = 0;
    }

    if (server.hasArg("mqtt_server"))
    {
      strncpy(mqtt_servername_param, server.arg("mqtt_server").c_str(), MAX_MQTT_SERVERNAME_LENGTH);
      mqtt_servername_param[MAX_MQTT_SERVERNAME_LENGTH] = 0;
    }
    if (server.hasArg("mqtt_id"))
    {
      strncpy(mqtt_sensorid_param, server.arg("mqtt_id").c_str(), MAX_MQTT_SENSORID_LENGTH);
      mqtt_sensorid_param[MAX_MQTT_SENSORID_LENGTH] = 0;
    }
    if (server.hasArg("mqtt_username"))
    {
      strncpy(mqtt_username_param, server.arg("mqtt_username").c_str(), MAX_MQTT_USERNAME_LENGTH);
      mqtt_username_param[MAX_MQTT_USERNAME_LENGTH] = 0;
    }
    if (server.hasArg("mqtt_password") && !server.arg("mqtt_password").equals(F("mqtt_password")))
    {
      strncpy(mqtt_password_param, server.arg("mqtt_password").c_str(), MAX_MQTT_PASSWORD_LENGTH);
      mqtt_password_param[MAX_MQTT_PASSWORD_LENGTH] = 0;
    }

    active_sensors_param = 0;
    if (server.hasArg("sensor0")) {active_sensors_param |= 1<<0;}
    if (server.hasArg("sensor1")) {active_sensors_param |= 1<<1;}
    if (server.hasArg("sensor2")) {active_sensors_param |= 1<<2;}
    if (server.hasArg("sensor3")) {active_sensors_param |= 1<<3;}
    if (server.hasArg("sensor4")) {active_sensors_param |= 1<<4;}
    if (server.hasArg("sensor5")) {active_sensors_param |= 1<<5;}
    if (server.hasArg("sensor6")) {active_sensors_param |= 1<<6;}
    if (server.hasArg("sensor7")) {active_sensors_param |= 1<<7;}

    writePersistentParams(ssid_param, password_param, active_sensors_param);

    server.send(200, F("text/plain"), F("Settings saved"));
  }
  else
  {
    readPersistentParams();

    String body = F("<!doctype html>"\
                    "<html lang=\"en\">"\
                    "<head>"\
                     "<meta charset=\"utf-8\">"\
                     "<title>Setup</title>"\
                     "<style>"\
                      "form {margin: 0.5em;}"\
                      "input {margin: 0.2em;}"\
                     "</style>"\
                    "</head>"\
                    "<body>"\
                     "<form method=\"post\">"\
                      "SSID:<input type=\"text\" name=\"ssid\" required maxlength=\"");
    body += String(MAX_SSID_LENGTH);
    body += F("\" autofocus value=\"");
    body += ssid_param;
    body += F("\"/><br/>"\
              "Password:<input type=\"password\" name=\"password\" maxlength=\"");
    body += String(MAX_PASSWORD_LENGTH);
    body += F("\" value=\"password\"/><br/><hr/>"\
              "MQTT server:<input type=\"text\" name=\"mqtt_server\" maxlength=\"");
    body += String(MAX_MQTT_SERVERNAME_LENGTH);
    body += F("\" value=\"");
    body += mqtt_servername_param;
    body += F("\"/><br/>"\
              "MQTT sensor id:<input type=\"text\" name=\"mqtt_id\" maxlength=\"");
    body += String(MAX_MQTT_SENSORID_LENGTH);
    body += F("\" value=\"");
    body += mqtt_sensorid_param;
    body += F("\"/><br/>"\
              "MQTT username:<input type=\"text\" name=\"mqtt_username\" maxlength=\"");
    body += String(MAX_MQTT_USERNAME_LENGTH);
    body += F("\" value=\"");
    body += mqtt_username_param;
    body += F("\"/><br/>"\
              "MQTT password:<input type=\"password\" name=\"mqtt_password\" maxlength=\"");
    body += String(MAX_MQTT_PASSWORD_LENGTH);
    body += F("\" value=\"password\"/><br/><hr/>"\
              "Internal temp/humidity sensor: <input type=\"checkbox\" name=\"sensor0\"");
    if (active_sensors_param&DHT_TEMP_HUMIDITY_SENSOR)
      body += F(" checked");

    body += F("/><br/>External temp sensor: <input type=\"checkbox\" name=\"sensor1\"");
    if (active_sensors_param&SHT_TEMP_HUMIDITY_SENSOR)
      body += F(" checked");
    
    body += F("/><br/>Pressure diff. sensor: <input type=\"checkbox\" name=\"sensor2\"");
    if (active_sensors_param&PRESSURE_DIFF_SENSOR)
      body += F(" checked");
    
    body += F("/><br/>Windspeed sensor: <input type=\"checkbox\" name=\"sensor3\"");
    if (active_sensors_param&WINDSPEED_SENSOR)
      body += F(" checked");
    
    body += F("/><br/>Fan relay: <input type=\"checkbox\" name=\"sensor4\"");
    if (active_sensors_param&FAN_RELAY)
      body += F(" checked");
    
    body += F("/><br/>Heater thermostat plug: <input type=\"checkbox\" name=\"sensor5\"");
    if (active_sensors_param&THERMOSTAT_RELAY)
      body += F(" checked");
    
    body += F("/><br/>"\
              "<input type=\"submit\" value=\"Submit\"/>"\
              "</form>"\
             "</body>"\
             "</html>");
    server.send(200, F("text/html"), body);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  int mqtt_sensorid_param_length = strlen(mqtt_sensorid_param);
  if (0 != strncmp(mqtt_sensorid_param, topic, mqtt_sensorid_param_length))
  {
    return;
  }

  printDebug((String("mqttCallback: ")+String(topic)+String(" value ")+String(reinterpret_cast<const char*>(payload))).c_str());
  const char* key = topic + mqtt_sensorid_param_length;

  if (0 == strcmp("Heater", key))
  {
    activateHeater(0 == strcmp("on", reinterpret_cast<const char*>(payload)));
  }
  else if (0 == strcmp("Fan", key))
  {
    activateFan(0 == strcmp("on", reinterpret_cast<const char*>(payload)));
  }
}

void publishMQTTValue(const String& topic, const String& msg)
{
  if (mqtt_enabled && connectMQTT())
  {
    bool ret = mqtt_client.publish((String(mqtt_sensorid_param)+topic).c_str(), msg.c_str(), true);
    printDebug((String("publishMQTTValue topic=")+String(mqtt_sensorid_param)+topic+String(" msg=")+msg+String(" returned ")+String(ret?"true":"false")).c_str());
  }
}

void publishMQTTValue(const String& topic, float value)
{
  publishMQTTValue(topic, String(value, 4));
}

void publishMQTTValues()
{
  if (!mqtt_enabled)
    return;

  if (active_sensors_param&DHT_TEMP_HUMIDITY_SENSOR)
  {
    if (dht_tempsensor_value!=published_dht_tempsensor_value)
    {
      publishMQTTValue(F("DHT22T"), dht_tempsensor_value);
      published_dht_tempsensor_value = dht_tempsensor_value;
    }
    if (dht_humiditysensor_value!=published_dht_humiditysensor_value)
    {
      publishMQTTValue(F("DHT22H"), dht_humiditysensor_value);
      published_dht_humiditysensor_value = dht_humiditysensor_value;
    }
  }

  if (active_sensors_param&SHT_TEMP_HUMIDITY_SENSOR)
  {
    if (sht_tempsensor_value!=published_sht_tempsensor_value)
    {
      publishMQTTValue(F("SHT2XT"), sht_tempsensor_value);
      published_sht_tempsensor_value = sht_tempsensor_value;
    }
    if (sht_humiditysensor_value!=published_sht_humiditysensor_value)
    {
      publishMQTTValue(F("SHT2XH"), sht_humiditysensor_value);
      published_sht_humiditysensor_value = sht_humiditysensor_value;
    }
  }

  if (active_sensors_param&PRESSURE_DIFF_SENSOR)
  {
    if (pressure_diff_value!=published_pressure_diff_value)
    {
      publishMQTTValue(F("Pressure"), pressure_diff_value);
      published_pressure_diff_value = pressure_diff_value;
    }
  }
  
  if (active_sensors_param&WINDSPEED_SENSOR)
  {
    float windspeed = getWindspeed();
    printDebug((String("reading wind speed value ")+String(windspeed, 2)).c_str());
    if (windspeed!=published_windspeed)
    {
      publishMQTTValue(F("Wind"), windspeed);
      published_windspeed = windspeed;
    }
  }
}

bool connectMQTT()
{
  byte i = 0;
  while (i++<10 && mqtt_enabled && !mqtt_client.connected())
  {
    if (mqtt_client.connect(mqtt_sensorid_param, mqtt_username_param, mqtt_password_param))
    {
      printDebug("MQTT connected");

      if (active_sensors_param & THERMOSTAT_RELAY)
      {
        mqtt_client.subscribe((String(mqtt_sensorid_param)+F("Heater")).c_str());
      }
      
      if (active_sensors_param & FAN_RELAY)
      {
        mqtt_client.subscribe((String(mqtt_sensorid_param)+F("Fan")).c_str());
      }
      
      printDebug("MQTT topics subscribed");
    }
    else
    {
      printDebug("MQTT waiting for reconnect");
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
  return mqtt_client.connected();
}

void setup()
{
  enableDebug();

  EEPROM.begin(1 + MAX_SSID_LENGTH+1 + MAX_PASSWORD_LENGTH+1 + MAX_ACTIVE_SENSORS_LENGTH + MAX_MQTT_SERVERNAME_LENGTH+1 + MAX_MQTT_SENSORID_LENGTH+1 + MAX_MQTT_USERNAME_LENGTH+1 + MAX_MQTT_PASSWORD_LENGTH+1);

  pinMode(I_SETUP_MODE_PIN, INPUT_PULLUP);
  delay(100);

  if (/*true ||*/ LOW == digitalRead(I_SETUP_MODE_PIN))
  {
    printDebug("Mode:Setup");

    mqtt_enabled = false;
    state = SETUP_MODE;
    WiFi.softAP(SETUP_SSID);
    dnsServer.start(DNS_PORT, "*", apIP);

    server.on("/", handleSetupRoot);
    server.onNotFound(handleNotFound);
    server.begin();
  }
  else
  {
    printDebug("Mode:Normal");

    dht.setup(I_DHT_PIN, DHTesp::AM2302);
    sht.begin(/*I_SHT_DATA_PIN, O_SHT_SYNC_PIN*/);

    pinMode(O_FAN_RELAY_TRIGGER_PIN, OUTPUT);
    activateFan(false);

    pinMode(O_THERMOSTAT_RELAY_TRIGGER_PIN, OUTPUT);
    activateHeater(false);

    readPersistentParams();
    mqtt_enabled = mqtt_servername_param && *mqtt_servername_param;
    
    printDebug("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_param, password_param);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
    }
    printDebug("WiFi connected");

    if (mqtt_enabled)
    {
      printDebug("Enabling MQTT");
      mqtt_client.setServer(mqtt_servername_param, 1883);
      mqtt_client.setCallback(mqttCallback);
      connectMQTT();
    }

    should_read_dht_temp_sensor = false;
    dht_tempsensor_value = dht_humiditysensor_value = 0.0f;

    should_read_sht_temp_sensor = false;
    sht_tempsensor_value = 0.0f;
    sht_humiditysensor_value = 0.0f;
    pressure_diff_value = 0.0f;

    state = READ_DHT_SENSOR;
    onTick();

    if (active_sensors_param && WINDSPEED_SENSOR)
    {
      windspeed_start_time = 0;
      windspeed_count = 0;
      attachInterrupt(digitalPinToInterrupt(I_WINDSPEED_PIN), windspeedInterruptHandler, RISING);
    }

    //Notify MQTT that relays have been turned off
    if (active_sensors_param & THERMOSTAT_RELAY)
    {
      publishMQTTValue("Heater", "off");
    }
    if (active_sensors_param & FAN_RELAY)
    {
      publishMQTTValue("Fan", "off");
    }
  }
}

void loop()
{
  if (state == SETUP_MODE)
  {
    dnsServer.processNextRequest(); //Route everything to 192.168.4.1
    server.handleClient(); //WebServer
  }
  else
  {
    if (mqtt_enabled && connectMQTT())
    {
      mqtt_client.loop();
    
      long now = millis();
      if ((now - last_mqtt_publish_time) > MQTT_PUBLISH_INTERVAL)
      {
        last_mqtt_publish_time = now;
        publishMQTTValues();
      }
    }

    //DHT22
    delay(max(1000, dht.getMinimumSamplingPeriod()));
    if ((active_sensors_param & DHT_TEMP_HUMIDITY_SENSOR) && should_read_dht_temp_sensor)
    {
      TempAndHumidity temp_and_humidity = dht.getTempAndHumidity();
      if (dht.getStatus() == DHTesp::ERROR_NONE)
      {
        should_read_dht_temp_sensor = false;
        dht_tempsensor_value = temp_and_humidity.temperature;
        dht_humiditysensor_value = temp_and_humidity.humidity;
        printDebug((String("reading temp ")+String(dht_tempsensor_value, 1)+String(" and humidity ")+String(dht_humiditysensor_value, 1)).c_str());
      }
      else
      {
        printDebug((String("reading temp and humidity failed with error ")+String((int)dht.getStatus())).c_str());
      }
    }
  
    //SHT2x
    if ((active_sensors_param & SHT_TEMP_HUMIDITY_SENSOR) && should_read_sht_temp_sensor)
    {
      sht_tempsensor_value = sht.getTemperature();
      sht_humiditysensor_value = sht.getHumidity();
      should_read_sht_temp_sensor = false;
    }
  }
}
