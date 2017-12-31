/*
 * Internet Of Låve
 * 
 * Measure and display Temperature and Humidity (AM2302 DHT22) and Wind Speed (MPXV7002DP), 
 * and provide these values to a Munin plugin.
 * 
 * Running on a WeMos D1 WiFi board.
 */

#include <DHT.h>
#include <TM1637Display.h>
#include <WiFi.h>


static const char ssid[] = "gill-roxrud";
static const char pass[] = "******";
WiFiServer server(80);

// 7-segment TM1637
static const byte TM1637_TEMP     = 0;
static const byte TM1637_HUMIDITY = 1;
static const byte TM1637_WIND     = 2;
static const byte SEGMENTS_DIO_PIN[] = {D2,D4,D6};
static const byte SEGMENTS_CLK_PIN[] = {D3,D5,D7};
static const byte SEGMENTS_BRIGHTNESS = 1; // 0-7
TM1637Display segment_displays[] = {TM1637Display(SEGMENTS_CLK_PIN[TM1637_TEMP], SEGMENTS_DIO_PIN[TM1637_TEMP]),
                                    TM1637Display(SEGMENTS_CLK_PIN[TM1637_HUMIDITY], SEGMENTS_DIO_PIN[TM1637_HUMIDITY]),
                                    TM1637Display(SEGMENTS_CLK_PIN[TM1637_WIND], SEGMENTS_DIO_PIN[TM1637_WIND])};

// Temperature/Humidity AM2302 DHT22
static const byte AM2302_PIN      = D8;
#define DHTTYPE (DHT22)
DHT dht(AM2302_PIN, DHTTYPE);

// Pressure MPXV7002DP
static const byte MPXV7002DP_PIN  = A0;

unsigned long previous_update_time = 0L;
static const int UPDATE_INTERVAL = 5000; //Should be >2000 because of DHT22


void readPressure(double& pressure) {
  static const int READ_COUNT = 10;
  double value = 0;
  for (int i=0; i<READ_COUNT; i++)
  {
    value += analogRead(MPXV7002DP_PIN); // 0-1023
  }
  value /= READ_COUNT;

  pressure = -2.5 + (5.0 * value/1023.0);
}

void readTemperatureHumidity(double& temperature, double& humidity)
{
  temperature = dht.readTemperature(); //Celsius
  humidity = dht.readHumidity();
}

void setSegments(TM1637Display& segment_display, byte b1, boolean decimal, byte b2)
{
  segment_display.showNumberDecEx(b1, decimal?0x30:0x00, false, 2, 0);
  segment_display.showNumberDec(b2, true, 2, 2);
}

void displayDouble(TM1637Display& segment_display, double value)
{
  int n = value;
  int d = (value-n)*100;
  setSegments(segment_display, n%100, true, d%100);
}

void setup()
{
  dht.begin();

  for (byte segment=TM1637_TEMP; segment<=TM1637_WIND; segment++)
  {
    segment_displays[segment].setBrightness(SEGMENTS_BRIGHTNESS);
  }

  WiFi.begin(const_cast<char*>(ssid), pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  server.begin();
}

void loop()
{
  WiFiClient client = server.available();

  unsigned long now = millis();
  if (client ||
      now<previous_update_time || //millis() wraps after ~50days
      (now-previous_update_time)>=UPDATE_INTERVAL)
  {
    double temperature;
    double humidity;
    readTemperatureHumidity(temperature, humidity);
  
    double pressure;
    readPressure(pressure);
  
    displayDouble(segment_displays[TM1637_TEMP], temperature);
    displayDouble(segment_displays[TM1637_HUMIDITY], humidity);
    displayDouble(segment_displays[TM1637_WIND], pressure);

    if (client)
    {
      if (client.available())
      {
        client.print("temperature: ");
        client.println(temperature);
        client.print("humidity: ");
        client.println(humidity);
        client.print("pressure: ");
        client.println(pressure);
        delay(10);
      }
      client.stop();
    }

    previous_update_time = now;
  }
  
  delay(25);
}

