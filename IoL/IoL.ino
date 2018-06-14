/*
 * Internet Of Låve
 * 
 * Measure and display Temperature and Humidity (AM2302 DHT22) and Wind Speed (MPXV7002DP), 
 * and provide these values to a Munin plugin.
 * 
 * Running on a WeMos D1 WiFi board.
 */

#include <DNSServer.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Ticker.h>

static const char* SETUP_SSID = "sensor-setup";
static const byte EEPROM_INITIALIZED_MARKER = 0xF1; //Just a magic number


#define SETUP_MODE_PIN              (D7)


enum State {
  SETUP_MODE,
};


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



void onTick()
{
  switch(state)
  {
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

void read_persistent_float(float& f, int decimals, int& adr)
{
  int i = 0;
  byte c;
  do {
    c = EEPROM.read(adr++);
    if (c>='0' && c<='9')
    {
      i = 10*i + (c-'0');
    }
  } while (c!=0);

  f = i;
  for (i=0; i<decimals; i++)
  {
    f = f/10.0f;
  }
}

void read_persistent_params()
{
  int adr = 0;
  if (EEPROM_INITIALIZED_MARKER != EEPROM.read(adr++))
  {
    ssid_param[0] = 0;
    password_param[0] = 0;
  } else {
    read_persistent_string(ssid_param, MAX_SSID_LENGTH, adr);
    read_persistent_string(password_param, MAX_PASSWORD_LENGTH, adr);
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

void write_persistent_params(const char* ssid, const char* password)
{
  int adr = 0;
  EEPROM.write(adr++, EEPROM_INITIALIZED_MARKER);
  write_persistent_string(ssid, MAX_SSID_LENGTH, adr);
  write_persistent_string(password, MAX_PASSWORD_LENGTH, adr);
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
    
    write_persistent_params(ssid_param, password_param);

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
  
    onTick();
  }
}

void loop()
{
  if (state == SETUP_MODE) {
      dnsServer.processNextRequest();
  }

  server.handleClient();
}
