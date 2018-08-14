
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
#include <WEMOS_SHT3X.h>
#include <Ticker.h>

static const char* SETUP_SSID = "sensor-setup";
static const byte  EEPROM_INITIALIZED_MARKER = 0xF1; //Just a magic number

static const uint8_t DHT_TEMP_HUMIDITY_SENSOR         = 1<<0;
static const uint8_t SHT_TEMP_HUMIDITY_SENSOR         = 1<<1;
static const uint8_t PRESSURE_DIFF_SENSOR             = 1<<2;
static const uint8_t WINDSPEED_SENSOR                 = 1<<3;
static const uint8_t FAN_RELAY                        = 1<<4;
static const uint8_t THERMOSTAT_PLUG                  = 1<<5;


#define PRESSURE_DIFF_PIN            (A0) //Max 3.3V

//#define (D0)                            //GPIO3 - Neither Interrupt, PWM, I2C nor One-wire
#define SHT_SYNC_PIN                 (D1) //GPIO1 - HW I2C
#define SHT_DATA_PIN                 (D2) //GPIO16 - HW I2C
#define DHT_PIN                      (D3) //GPIO5 - 10K pull-up
#define SETUP_MODE_PIN               (D4) //GPIO4 - 10K pull-up, built-in LED
//#define (D5) //GPIO14 - 
#define WINDSPEED_PIN                (D6) //GPIO12 - 
#define FAN_RELAY_TRIGGER_PIN        (D7) //GPIO13 - 
#define THERMOSTAT_PLUG_TRIGGER_PIN  (D8) //GPIO0 - 10K pull-down
//#define (D9)                            //GPIO2 - 
//#define (D10)                           //GPIO15 - 

#define DELAY_BETWEEN_ACTIVE_SENSORS (25) //ms between reading different sensors
#define SENSOR_READ_DELAY_TIME       (10*1000)  //ms min. time between sensor reading cycle

static const float WINDSPEED_TRAVEL_DISTANCE = 0.2262; //meter pr interrupt trigger signal


enum State {
  SETUP_MODE,
  READ_DHT_SENSOR,
  READ_SHT_SENSOR,
  READ_PRESSURE_DIFF_SENSOR
};


DHTesp dht;
SHT3X sht(0x45);

ESP8266WebServer server(80);

WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);

DNSServer dnsServer;
static const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

Ticker ticker;

volatile State state;

#define MAX_SSID_LENGTH            (32)
#define MAX_PASSWORD_LENGTH        (64)
#define MAX_MQTT_SERVERNAME_LENGTH (64)
#define MAX_MQTT_USERNAME_LENGTH   (32)
#define MAX_MQTT_PASSWORD_LENGTH   (32)

char ssid_param[MAX_SSID_LENGTH+1];
char password_param[MAX_PASSWORD_LENGTH+1];
char mqtt_servername_param[MAX_MQTT_SERVERNAME_LENGTH+1];
char mqtt_username_param[MAX_MQTT_USERNAME_LENGTH+1];
char mqtt_password_param[MAX_MQTT_PASSWORD_LENGTH+1];
uint8_t active_sensors_param = 0;

volatile bool should_read_dht_temp_sensor;
float dht_tempsensor_value;
float dht_humiditysensor_value;

volatile bool should_read_sht_temp_sensor;
float sht_tempsensor_value;
float sht_humiditysensor_value;

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
    case READ_DHT_SENSOR:
      {
        if (active_sensors_param & DHT_TEMP_HUMIDITY_SENSOR) {
          should_read_dht_temp_sensor = true;
        }

        state = READ_SHT_SENSOR;
        ticker.attach_ms(DELAY_BETWEEN_ACTIVE_SENSORS, onTick);
        break;  
      }

    case READ_SHT_SENSOR:
      {
        if (active_sensors_param & SHT_TEMP_HUMIDITY_SENSOR) {
          should_read_sht_temp_sensor = true;
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
        
        state = READ_DHT_SENSOR;
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
    mqtt_servername_param[0] = 0;
    mqtt_username_param[0] = 0;
    mqtt_password_param[0] = 0;
    active_sensors_param = 0;
  } else {
    read_persistent_string(ssid_param, MAX_SSID_LENGTH, adr);
    read_persistent_string(password_param, MAX_PASSWORD_LENGTH, adr);
    read_persistent_byte(active_sensors_param, adr);
    read_persistent_string(mqtt_servername_param, MAX_MQTT_SERVERNAME_LENGTH, adr);
    read_persistent_string(mqtt_username_param, MAX_MQTT_USERNAME_LENGTH, adr);
    read_persistent_string(mqtt_password_param, MAX_MQTT_PASSWORD_LENGTH, adr);
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
  write_persistent_string(mqtt_servername_param, MAX_MQTT_SERVERNAME_LENGTH, adr);
  write_persistent_string(mqtt_username_param, MAX_MQTT_USERNAME_LENGTH, adr);
  write_persistent_string(mqtt_password_param, MAX_MQTT_PASSWORD_LENGTH, adr);
  EEPROM.commit();
}

void handleSensorsConfig() {
  String response = F("graph_title InternetOfLåve\n"\
                      "graph_args --base 1000 -l 0\n"\
                      "graph_vlabel Value\n"\
                      "graph_category homeautomation\n"\
                      "graph_info IoL sensor values.\n");

  if (active_sensors_param&DHT_TEMP_HUMIDITY_SENSOR)
    response += F("AIRTEMP.label Air Temperature\n"\
                  "AIRTEMP.draw LINE2\n"\
                  "AIRTEMP.info C\n"\
                  "AIRHUMIDITY.label Air Humidity\n"\
                  "AIRHUMIDITY.draw LINE2\n"\
                  "AIRHUMIDITY.info RH\n");

  if (active_sensors_param&SHT_TEMP_HUMIDITY_SENSOR)
    response += F("GRAINTEMP.label Grain Temperature\n"\
                  "GRAINTEMP.draw LINE2\n"\
                  "GRAINTEMP.info C\n"\
                  "GRAINHUMIDITY.label Grain Humidity\n"\
                  "GRAINHUMIDITY.draw LINE2\n"\
                  "GRAINHUMIDITY.info RH\n");
  
  if (active_sensors_param&PRESSURE_DIFF_SENSOR)
    response += F("AIRPRESSURE.label Air Pressure Difference\n"\
                  "AIRPRESSURE.draw LINE2\n"\
                  "AIRPRESSURE.info KPa\n");
  
  if (active_sensors_param&WINDSPEED_SENSOR)
    response += F("WIND.label Wind Speed\n"\
                  "WIND.draw LINE2\n"\
                  "WIND.info m/s\n");
  
  if (active_sensors_param&FAN_RELAY)
    response += F("FAN.label Fan Relay\n"\
                  "FAN.draw LINE2\n"\
                  "FAN.info active\n");
  
  if (active_sensors_param&THERMOSTAT_PLUG)
    response += F("THERMOSTAT.label Thermostat Plug\n"\
                  "THERMOSTAT.draw LINE2\n"\
                  "THERMOSTAT.info active\n");

  server.send(200, F("text/plain"), response);

}

void handleSensorsValues() {
  String response;
  
  if (active_sensors_param&DHT_TEMP_HUMIDITY_SENSOR)
  {
    response += F("AIRTEMP.value ");
    response += dht_tempsensor_value;
    response += F("\nAIRHUMIDITY.value ");
    response += dht_humiditysensor_value;
    response += F("\n");
  }

  if (active_sensors_param&SHT_TEMP_HUMIDITY_SENSOR)
  {
    response += F("GRAINTEMP.value ");
    response += sht_tempsensor_value;
    response += F("\nGRAINHUMIDITY.value ");
    response += sht_humiditysensor_value;
    response += F("\n");
  }
  
  if (active_sensors_param&PRESSURE_DIFF_SENSOR)
    response += F("AIRPRESSURE.label Air Pressure Difference\n"\
                  "AIRPRESSURE.draw LINE2\n"\
                  "AIRPRESSURE.info KPa\n");
  
  if (active_sensors_param&WINDSPEED_SENSOR)
  {
    response += F("WIND.value ");
    response += getWindspeed();
    response += F("\n");
  }
  
  if (active_sensors_param&FAN_RELAY)
    response += F("FAN.label Fan Relay\n"\
                  "FAN.draw LINE2\n"\
                  "FAN.info active\n");
  
  if (active_sensors_param&THERMOSTAT_PLUG)
    response += F("THERMOSTAT.label Thermostat Plug\n"\
                  "THERMOSTAT.draw LINE2\n"\
                  "THERMOSTAT.info active\n");

  server.send(200, F("text/plain"), response);
}

void handleHeaterActivate() {
  if (active_sensors_param & THERMOSTAT_PLUG) {
    digitalWrite(THERMOSTAT_PLUG_TRIGGER_PIN, HIGH);
  }
}

void handleHeaterDeactivate() {
  if (active_sensors_param & THERMOSTAT_PLUG) {
    digitalWrite(THERMOSTAT_PLUG_TRIGGER_PIN, LOW);
  }
}

void handleFanActivate() {
  if (active_sensors_param & FAN_RELAY) {
    digitalWrite(FAN_RELAY_TRIGGER_PIN, HIGH);
  }
}

void handleFanDeactivate() {
  if (active_sensors_param & FAN_RELAY) {
    digitalWrite(FAN_RELAY_TRIGGER_PIN, LOW);
  }
}

void handleNotFound() {
  server.send(404, F("text/plain"), F("Page Not Found\n"));
}

void handleSetupRoot() {
  if (server.hasArg("ssid") || server.hasArg("password")
      || server.hasArg("mqtt_server") || server.hasArg("mqtt_username") || server.hasArg("mqtt_password")
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
    body += F("\" value=\"password\"/><br/><hr/>"\
              "MQTT server:<input type=\"text\" name=\"mqtt_server\" maxlength=\"");
    body += String(MAX_MQTT_SERVERNAME_LENGTH);
    body += F("\" value=\"");
    body += mqtt_servername_param;
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

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
}

void reconnect_mqtt() {
  while (!mqtt_client.connected()) {
    if (mqtt_client.connect("ESP8266Client")) {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  EEPROM.begin(1 + MAX_SSID_LENGTH + 1 + MAX_PASSWORD_LENGTH + 1);

  dht.setup(DHT_PIN, DHTesp::AM2302);

  pinMode(SETUP_MODE_PIN, INPUT_PULLUP);
  
  pinMode(FAN_RELAY_TRIGGER_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_TRIGGER_PIN, LOW);

  pinMode(THERMOSTAT_PLUG_TRIGGER_PIN, OUTPUT);
  digitalWrite(THERMOSTAT_PLUG_TRIGGER_PIN, LOW);

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
  
    mqtt_client.setServer(mqtt_servername_param, 1883);
    mqtt_client.setCallback(mqtt_callback);

    server.on(F("/sensors/config"), handleSensorsConfig);
    server.on(F("/sensors/values"), handleSensorsValues);
    server.on(F("/heater/activate"), handleHeaterActivate);
    server.on(F("/heater/deactivate"), handleHeaterDeactivate);
    server.on(F("/fan/activate"), handleFanActivate);
    server.on(F("/fan/deactivate"), handleFanDeactivate);
    server.onNotFound(handleNotFound);
    server.begin();
  
    should_read_dht_temp_sensor = false;
    dht_tempsensor_value = dht_humiditysensor_value = 0.0f;

    should_read_sht_temp_sensor = false;
    sht_tempsensor_value = 0.0f;
    sht_humiditysensor_value = 0.0f;

    state = READ_DHT_SENSOR;
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

  if (!mqtt_client.connected()) {
    reconnect_mqtt();
  }
  mqtt_client.loop();

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

  if ((active_sensors_param & SHT_TEMP_HUMIDITY_SENSOR) && should_read_sht_temp_sensor) {
    if (0 == sht.get()) {
      should_read_sht_temp_sensor = false;
      sht_tempsensor_value = sht.cTemp;
      sht_humiditysensor_value = sht.humidity;
    }
  }
}
