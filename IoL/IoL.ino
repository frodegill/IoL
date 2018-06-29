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
#include "Sensirion.h"
#include <Ticker.h>

static const char* SETUP_SSID = "sensor-setup";
static const byte EEPROM_INITIALIZED_MARKER = 0xF1; //Just a magic number

static const uint8_t DHT_TEMP_HUMIDITY_SENSOR        = 1<<0;
static const uint8_t PRECISION_TEMP_HUMIDITY_SENSOR1 = 1<<1;
static const uint8_t PRECISION_TEMP_HUMIDITY_SENSOR2 = 1<<2;
static const uint8_t PRESSURE_DIFF_SENSOR            = 1<<3;
static const uint8_t WINDSPEED_SENSOR                = 1<<4;
static const uint8_t FAN_RELAY                       = 1<<5;
static const uint8_t THERMOSTAT_PLUG                 = 1<<6;


#define PRESSURE_DIFF_PIN            (A0) //Max 3.3V
//#define (TX)
//#define (RX)
//#define (D0) //Neither Interrupt, PWM, I2C nor One-wire
#define DHT_PIN                      (D1) //HW I2C
#define PRECISION_TEMP1_DATA_PIN     (D2) //HW I2C
#define PRECISION_TEMP2_DATA_PIN     (D3) //10K pull-up
#define SETUP_MODE_PIN               (D4) //10K pull-up, built-in LED
#define PRECISION_TEMP_SYNC_PIN      (D5)
#define WINDSPEED_PIN                (D6)
#define FAN_RELAY_TRIGGER_PIN        (D7)
#define THERMOSTAT_PLUG_TRIGGER_PIN  (D8) //10K pull-down

#define DELAY_BETWEEN_ACTIVE_SENSORS (25) //ms between reading different sensors
#define SENSOR_READ_DELAY_TIME       (10*1000)  //ms min. time between sensor reading cycle

static const float WINDSPEED_TRAVEL_DISTANCE = 0.22; //meter pr interrupt trigger signal


enum State {
  SETUP_MODE,
  READ_TEMP_HUMIDITY_SENSOR,
  READ_PRECISION_TEMP1_SENSOR,
  READ_PRECISION_TEMP2_SENSOR,
  READ_PRESSURE_DIFF_SENSOR
};


DHTesp dht;
Sensirion sht1 = Sensirion(PRECISION_TEMP1_DATA_PIN, PRECISION_TEMP_SYNC_PIN, 0x40);
Sensirion sht2 = Sensirion(PRECISION_TEMP2_DATA_PIN, PRECISION_TEMP_SYNC_PIN, 0x40);

ESP8266WebServer server(80);
Ticker ticker;

DNSServer dnsServer;
static const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

volatile State state;

#define MAX_SSID_LENGTH        (32)
#define MAX_PASSWORD_LENGTH    (64)

char ssid_param[MAX_SSID_LENGTH+1];
char password_param[MAX_PASSWORD_LENGTH+1];
uint8_t active_sensors_param = 0;

volatile bool should_read_dht_temp_sensor;
float dht_tempsensor_value;
float dht_humiditysensor_value;

volatile bool should_read_sht_temp_sensor[2];
float sht_tempsensor_value[2];
float sht_humiditysensor_value[2];

volatile unsigned long windspeed_start_time;
volatile unsigned int windspeed_count;


void windspeedInterruptHandler() {
  if (0 == windspeed_count) {
    windspeed_start_time = millis();
  }
  windspeed_count++;
}

float getWindspeed() { // m/s
  if (0 == windspeed_count) {
    return 0.0f;
  }

  unsigned long current_time = millis();
  if (current_time < windspeed_start_time) {
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
    case READ_TEMP_HUMIDITY_SENSOR:
      {
        if (active_sensors_param & DHT_TEMP_HUMIDITY_SENSOR) {
          should_read_dht_temp_sensor = true;
        }

        state = READ_PRECISION_TEMP1_SENSOR;
        ticker.attach_ms(DELAY_BETWEEN_ACTIVE_SENSORS, onTick);
        break;  
      }

    case READ_PRECISION_TEMP1_SENSOR:
      {
        if (active_sensors_param & PRECISION_TEMP_HUMIDITY_SENSOR1) {
          should_read_sht_temp_sensor[0] = true;
        }
        
        state = READ_PRECISION_TEMP2_SENSOR;
        ticker.attach_ms(DELAY_BETWEEN_ACTIVE_SENSORS, onTick);
        break;
      }
    
    case READ_PRECISION_TEMP2_SENSOR:
      {
        if (active_sensors_param & PRECISION_TEMP_HUMIDITY_SENSOR2) {
          should_read_sht_temp_sensor[1] = true;
        }

        state = READ_PRESSURE_DIFF_SENSOR;
        ticker.attach_ms(DELAY_BETWEEN_ACTIVE_SENSORS, onTick);
        break;
      }
    
    case READ_PRESSURE_DIFF_SENSOR:
      {
        if (active_sensors_param & READ_PRESSURE_DIFF_SENSOR) {
          int value = max(0, min(1023, analogRead(PRESSURE_DIFF_PIN)));
        }
        
        state = READ_PRECISION_TEMP1_SENSOR;
        ticker.attach_ms(SENSOR_READ_DELAY_TIME, onTick);
        break;
      }
    
    default: break;
  }
}

void read_persistent_string(char* s, int max_length, int& adr)
{
  int i = 0;
  byte c;
  do {
    c = EEPROM.read(adr++);
    if (i<max_length)
    {
      s[i++] = static_cast<char>(c);
    }
  } while (c!=0);
  s[i] = 0;
}

void read_persistent_byte(uint8_t& b, int& adr)
{
  b = EEPROM.read(adr++);
}

void read_persistent_params()
{
  int adr = 0;
  if (EEPROM_INITIALIZED_MARKER != EEPROM.read(adr++))
  {
    ssid_param[0] = 0;
    password_param[0] = 0;
    active_sensors_param = 0;
  } else {
    read_persistent_string(ssid_param, MAX_SSID_LENGTH, adr);
    read_persistent_string(password_param, MAX_PASSWORD_LENGTH, adr);
    read_persistent_byte(active_sensors_param, adr);
  }
}

void write_persistent_string(const char* s, size_t max_length, int& adr)
{
  for (int i=0; i<min(strlen(s), max_length); i++)
  {
    EEPROM.write(adr++, s[i]);
  }
  EEPROM.write(adr++, 0);
}

void write_persistent_byte(uint8_t b, int& adr)
{
  EEPROM.write(adr++, b);
}

void write_persistent_params(const char* ssid, const char* password, uint8_t active_sensors)
{
  int adr = 0;
  EEPROM.write(adr++, EEPROM_INITIALIZED_MARKER);
  write_persistent_string(ssid, MAX_SSID_LENGTH, adr);
  write_persistent_string(password, MAX_PASSWORD_LENGTH, adr);
  write_persistent_byte(active_sensors, adr);
  EEPROM.commit();
}

void handleCountConfig() {
  server.send(200, F("text/plain"), F("graph_title InternetOfLåve\n"\
                                      "graph_args --base 1000 -l 0\n"\
                                      "graph_vlabel Count\n"\
                                      "graph_category homeautomation\n"));
}

void handleCountValues() {
  char msg[20];
  snprintf(msg, sizeof(msg)/sizeof(msg[0]), "\n");
  server.send(200, F("text/plain"), msg);
}

void handleSensorsConfig() {
  server.send(200, F("text/plain"), F("graph_title InternetOfLåve\n"\
                                      "graph_args --base 1000 -l 0\n"\
                                      "graph_vlabel Value\n"\
                                      "graph_category homeautomation\n"\
                                      "graph_info IoL sensor values.\n"));
}

void handleSensorsValues() {
  char msg[80];
  snprintf(msg, sizeof(msg)/sizeof(msg[0]), "\n");
  server.send(200, F("text/plain"), msg);
}

void handleNotFound() {
  server.send(404, F("text/plain"), F("Page Not Found\n"));
}

void handleSetupRoot() {
  if (server.hasArg("ssid") || server.hasArg("password"))
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

    active_sensors_param = 0;
    if (server.hasArg("sensor0")) {active_sensors_param |= 1<<0;}
    if (server.hasArg("sensor1")) {active_sensors_param |= 1<<1;}
    if (server.hasArg("sensor2")) {active_sensors_param |= 1<<2;}
    if (server.hasArg("sensor3")) {active_sensors_param |= 1<<3;}
    if (server.hasArg("sensor4")) {active_sensors_param |= 1<<4;}
    if (server.hasArg("sensor5")) {active_sensors_param |= 1<<5;}
    if (server.hasArg("sensor6")) {active_sensors_param |= 1<<6;}
    if (server.hasArg("sensor7")) {active_sensors_param |= 1<<7;}

    write_persistent_params(ssid_param, password_param, active_sensors_param);

    server.send(200, F("text/plain"), F("Settings saved"));
  }
  else
  {
    read_persistent_params();

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
    body += F("\" value=\"password\"/><br/>"\
                      "Internal temp/humidity sensor: <input type=\"checkbox\" name=\"sensor0\"");
    if (active_sensors_param&DHT_TEMP_HUMIDITY_SENSOR)
      body += F(" checked");

    body += F("/><br/>External temp sensor 1: <input type=\"checkbox\" name=\"sensor1\"");
    if (active_sensors_param&PRECISION_TEMP_HUMIDITY_SENSOR1)
      body += F(" checked");
    
    body += F("/><br/>External temp sensor 2: <input type=\"checkbox\" name=\"sensor2\"");
    if (active_sensors_param&PRECISION_TEMP_HUMIDITY_SENSOR2)
      body += F(" checked");
    
    body += F("/><br/>Pressure diff. sensor: <input type=\"checkbox\" name=\"sensor3\"");
    if (active_sensors_param&PRESSURE_DIFF_SENSOR)
      body += F(" checked");
    
    body += F("/><br/>Windspeed sensor: <input type=\"checkbox\" name=\"sensor4\"");
    if (active_sensors_param&WINDSPEED_SENSOR)
      body += F(" checked");
    
    body += F("/><br/>Fan relay: <input type=\"checkbox\" name=\"sensor5\"");
    if (active_sensors_param&FAN_RELAY)
      body += F(" checked");
    
    body += F("/><br/>Heater thermostat plug: <input type=\"checkbox\" name=\"sensor6\"");
    if (active_sensors_param&THERMOSTAT_PLUG)
      body += F(" checked");
    
    body += F("/><br/>"\
              "<input type=\"submit\" value=\"Submit\"/>"\
              "</form>"\
             "</body>"\
             "</html>");
    server.send(200, F("text/html"), body);
  }
}

void setup()
{
  EEPROM.begin(1 + MAX_SSID_LENGTH + 1 + MAX_PASSWORD_LENGTH + 1);

  dht.setup(DHT_PIN, DHTesp::AUTO_DETECT);

  pinMode(SETUP_MODE_PIN, INPUT_PULLUP);

  if (LOW == digitalRead(SETUP_MODE_PIN))
  {
    state = SETUP_MODE;
    WiFi.softAP(SETUP_SSID);
    dnsServer.start(DNS_PORT, "*", apIP);

    server.on("/", handleSetupRoot);
    server.begin();
  }
  else
  {
    read_persistent_params();
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_param, password_param);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
  
    server.on(F("/count/config"), handleCountConfig);
    server.on(F("/count/values"), handleCountValues);
    server.on(F("/sensors/config"), handleSensorsConfig);
    server.on(F("/sensors/values"), handleSensorsValues);
    server.onNotFound(handleNotFound);
    server.begin();
  
    should_read_dht_temp_sensor = false;
    dht_tempsensor_value = dht_humiditysensor_value = 0.0f;

    should_read_sht_temp_sensor[0] = should_read_sht_temp_sensor[1] = false;
    sht_tempsensor_value[0] = sht_tempsensor_value[1] = 0.0f;
    sht_humiditysensor_value[0] = sht_humiditysensor_value[1] = 0.0f;

    state = READ_TEMP_HUMIDITY_SENSOR;
    onTick();

    if (active_sensors_param && WINDSPEED_SENSOR) {
      windspeed_start_time = 0;
      windspeed_count = 0;
      attachInterrupt(digitalPinToInterrupt(WINDSPEED_PIN), windspeedInterruptHandler, RISING);
    }
  }
}

void loop()
{
  if (state == SETUP_MODE) {
      dnsServer.processNextRequest();
  }

  server.handleClient();

  if ((active_sensors_param & DHT_TEMP_HUMIDITY_SENSOR) && should_read_dht_temp_sensor) {
    should_read_dht_temp_sensor = false;
    delay(dht.getMinimumSamplingPeriod());
    TempAndHumidity temp_and_humidity = dht.getTempAndHumidity();
    if (dht.getStatus() == DHTesp::ERROR_NONE)
    {
      dht_tempsensor_value = temp_and_humidity.temperature;
      dht_humiditysensor_value = temp_and_humidity.humidity;
    }
  }

  if ((active_sensors_param & PRECISION_TEMP_HUMIDITY_SENSOR1) && should_read_sht_temp_sensor[0]) {
    float tmp_temp, tmp_humidity;
    if (S_Meas_Rdy == sht1.measure(&tmp_temp, &tmp_humidity)) {
      should_read_sht_temp_sensor[0] = false;
      sht_tempsensor_value[0] = tmp_temp;
      sht_humiditysensor_value[0] = tmp_humidity;
    }
  }

  if ((active_sensors_param & PRECISION_TEMP_HUMIDITY_SENSOR2) && should_read_sht_temp_sensor[1]) {
    float tmp_temp, tmp_humidity;
    if (S_Meas_Rdy == sht2.measure(&tmp_temp, &tmp_humidity)) {
      should_read_sht_temp_sensor[1] = false;
      sht_tempsensor_value[1] = tmp_temp;
      sht_humiditysensor_value[1] = tmp_humidity;
    }
  }
}
