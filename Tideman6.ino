

#include <esp_err.h>
#include <nvs_flash.h>

#include "DFRobot_AHT20.h"
DFRobot_AHT20 aht20;


#include "Arduino.h"

//BME680
#include "bme68xLibrary.h"

Bme68x bme;




//includes
#include <WiFi.h>
#include <WiFiUDP.h>
#include <map>
#include <algorithm>
#include <string.h>
#include <Wire.h>


const uint64_t sleepTime = 120e6; // 5 minutes in microseconds

uint64_t lastLoopTime=millis();


//BATTERY CALC  SCALE over time.
float highestVoltage=0.0;
float lowestVoltage=100.0;

#define PI 3.14159



//CODE
String Version = "Tideman 6 DFrobot Firebeetle 2 ESP32-E Gravity IO Shield bme680";                       // Version 
String BoardId = "windman.ktxcypress-900";   










unsigned long lastTime = millis(); // Milliseconds since last measurement
unsigned long sampleTime = 1000;   // Sample time in milliseconds (1 second)
const int batteryPin = A2; // Analog pin connected to the battery voltage

//MY WIFI

const char* ssid     = "cam24";
const char* password = "olivia15";

//const char* ssid     = "YoDaddy";
//const char* password = "abcd1234";

//UDP
int port = 8089;
const char *influxDNS = "bi.pancamo.com";
IPAddress influxIP;
WiFiUDP udp;


//OTA
#include <ElegantOTA.h> 
#include <WiFiClient.h>
#include <WebServer.h>
WebServer webserver(80);

//WIFIMAN
uint8_t DisconnectReason=0;
unsigned long wifiUptime = millis();
unsigned long wifiDowntime = millis();

String line;





// Sonar
float SENSORHEIGHTINFEET=61/12;  // The hight of the botton of the sensor in feet

bool newData = false; // Whether new data is available from the sensor
uint8_t buffer[4];  // our buffer for storing data
uint8_t idx = 0;  // our idx into the storage buffer
float avg;
float distance;  // The last measured distance
float distlow;  // The lowest measured distance
float disthigh;  // The highest measured distance
float distmean;  // The middle measured distance
int count = 0;
int loopcnt = 0;
float mDistance[10000];


float WaterLevelHigh;
float WaterLevelLow;
float WaterLevelAvg;



//Normalize Data, remove the bad data with 5 samples
std::map<String, int> data[5];
// Function prototype
int medianData(std::map<String, int> (&data)[5], String code);

int  medianData(std::map<String, int> (&data)[5], String code) {
  // Access the values of the map array
  int numbers[5];

  for (int i = 0; i < 5; i++) {
    numbers[i] = data[i][code];
  }
  std::sort(numbers, numbers + 5);
  return (numbers[2]);
}



int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

float average (float * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}





//WIFIMAN
void wifiSetup()
{

  // WIFI RECONNECT
  WiFi.disconnect(true);
  wifiUptime=millis();
  wifiDowntime=millis();
  
  delay(2000);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);


  WiFi.begin(ssid, password);
    
  Serial.println();
  Serial.println();
  Serial.println("Wait for WiFi... ");
  
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  DisconnectReason = info.wifi_sta_disconnected.reason;

  wifiDowntime=millis();

  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected OTA");
  Serial.print ("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Version: " + Version);

  if (WiFi.hostByName(influxDNS, influxIP)) {
      Serial.print("Influx IP: ");
      Serial.println(influxIP);
    } else {
      Serial.println("DNS lookup failed for " + String(influxDNS));
    }
  
  wifiDowntime=millis();
  
  // reason 0 on 1st status is ok
  line = String(BoardId + ".wifi.disreason value=" + String(DisconnectReason));
  toInflux(line);  

 
  String ipAddressString = WiFi.localIP().toString();

    
 
  //line = String(BoardId + ".wifi.localip, tag1=value1, ip_address=") + String(ipAddressString) + String(" ") + String(" value=10");

  // line = String("testip,tag1=value1 ip_address=123 value=20") ;
  // toInflux(line);

}








void logWifiStatus()
{
  line = String(BoardId + ".wifi.rssi value=" + String(WiFi.RSSI()));
  toInflux(line);

  wifiUptime=millis()-wifiDowntime; 
  line = String(BoardId + ".wifi.uptime value=" + String(wifiUptime/1000));
  toInflux(line);

  line = String(BoardId + ".wifi.ip" + ",ipaddress=" + WiFi.localIP().toString() + " value=75") ;

  toInflux(line);

}

//OTA 
void handle_OnConnect() 
{
  webserver.send(200, "text/plain", "Hello from " + Version + " " + BoardId);
}


void otaSetup()
{
  webserver.on("/", handle_OnConnect);
  ElegantOTA.begin(&webserver);    // Start ElegantOTA
  ElegantOTA. setAutoReboot(true);
  webserver.begin();
}




//UDP
void toInflux (String line)
{

      Serial.println(line);

      udp.beginPacket(influxIP, port);
      udp.print(line);
      udp.endPacket();

}



//NVS FLASH
// Function to write a float value to NVS flash
esp_err_t writeNVSFloat(const char* var_name, float value) {
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open NVS handle
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Write float to NVS
  err = nvs_set_blob(my_handle, var_name, &value, sizeof(value));

  // Commit updates and close handle
  if (err == ESP_OK) {
    err = nvs_commit(my_handle);
  }
  nvs_close(my_handle);

  return err;
}

// Function to read a float value from NVS flash
float readNVSFloat(const char* var_name) {
  nvs_handle_t my_handle;
  esp_err_t err;
  float value = -1.0f;  // Default value in case of error

  // Open NVS handle
  err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    return value;  // Return default value on error
  }

  // Read float from NVS
  size_t data_size = sizeof(value);
  err = nvs_get_blob(my_handle, var_name, &value, &data_size);

  // Close handle
  nvs_close(my_handle);

  // Check for errors or data size mismatch
  if (err != ESP_OK || data_size != sizeof(value)) {
    return value;  // Return default value on error
  }

  return value;
}


void setupNVS() {
  

  // Initialize NVS flash
  int err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    // If no free pages, perform NVS erase (optional)
    Serial.println("NVS flash full, erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);  // Check for other errors

  Serial.println("ESP32 NVS flash ready");

  // Read initial battery level from NVS (optional)
  highestVoltage = readNVSFloat("highestVoltage");
  if (highestVoltage != -1.0f) {
    Serial.println("BATTERY: highestVoltage Saved battery level: " + String(highestVoltage));
  } else {
    Serial.println("No highestVoltage battery level stored in NVS");
  }

  lowestVoltage = readNVSFloat("lowestVoltage");
  if (lowestVoltage != -1.0f) {
    Serial.println("BATTERY: lowestVoltage Saved battery level: "+ String(lowestVoltage));
  } else {
    Serial.println("No lowestVoltage battery level stored in NVS");
  }


}










void setupAHT20(){

    uint8_t status;
    int failCnt=0;
    while((status = aht20.begin()) != 0)
    {
      Serial.print("AHT20 sensor initialization failed. error status : ");
      Serial.println(status);
      delay(500);
      failCnt++;
      if (failCnt > 3) {break;}
    }


}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // 
  //  (X - 1.5) * (100-0) / (1.9-1.5) + 0 
}



float logBatteryLevel() {
  // Read the battery voltage (assuming a 3.7V LiPo battery)
  float voltage = analogRead(batteryPin) * (3.26 / 4095.0);  

  //Serial.println("Voltage=" + String(voltage));


  //UPDATE NVS with highest and lowest measured voltages

  if (voltage > highestVoltage) {
    highestVoltage=voltage;
    writeNVSFloat("highestVoltage", highestVoltage);
    }

  if (voltage < lowestVoltage && voltage > 0) {
    lowestVoltage=voltage;
    writeNVSFloat("lowestVoltage", lowestVoltage);
    }


  // Map the voltage to a percentage (replace with your specific battery curve if known)
  float percentage = mapFloat(voltage, lowestVoltage, highestVoltage, 0, 100);

  // Constrain the percentage to be between 0 and 100
  // percentage = constrain(percentage, 0, 100);


  toInflux(BoardId + ".battery.level value=" + String(percentage));
  toInflux(BoardId + ".battery.voltage value=" + String(voltage));
  toInflux(BoardId + ".battery.highestVoltage value=" + String(highestVoltage));
  toInflux(BoardId + ".battery.lowestVoltage value=" + String(lowestVoltage));



  return percentage;
}






#define PI 3.14159  // Define pi for circumference calculation





void logTemperature()
{


  if(aht20.startMeasurementReady(true))
  {

      //AHT20
      float temperature = aht20.getTemperature_F();
      float humidity = aht20.getHumidity_RH();

      //Serial.printf("Temp = %2.2f\n", temperature);   

      line = String(BoardId + ".aht20.temperature value=" + String(temperature));
      toInflux(line);
      line = String(BoardId + ".aht20.humidity value=" + String(humidity));
      toInflux(line);
  }

}









void setupBME680()
{
  Wire.begin();     //I2C mode
	//SPI.begin();    //SPI mode
	
	while (!Serial)
		delay(10);
		
	/* initializes the sensor based on SPI library */
	//bme.begin(PIN_CS, SPI);     //SPI mode
  bme.begin(ADD_I2C, Wire);     //I2C mode

	if(bme.checkStatus())
	{
		if (bme.checkStatus() == BME68X_ERROR)
		{
			Serial.println("Sensor error:" + bme.statusString());
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING)
		{
			Serial.println("Sensor Warning:" + bme.statusString());
		}
	}
	
	/* Set the default configuration for temperature, pressure and humidity */
	bme.setTPH();

	/* Set the heater configuration to 300 deg C for 100ms for Forced mode */
	bme.setHeaterProf(300, 100);

	//Serial.println("TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%), Gas resistance(ohm), Status");
}

void logBME680(void)
{
	bme68xData data;

	bme.setOpMode(BME68X_FORCED_MODE);
	//delay(500+bme.getMeasDur()/200);

	if (bme.fetchData())
	{
		bme.getData(data);

      line = String(BoardId + ".bme680.temperature value=" + String((data.temperature * 9/5) + 32));
      toInflux(line);
      line = String(BoardId + ".bme680.humidity value=" + String(data.humidity));
      toInflux(line);
      line = String(BoardId + ".bme680.pressure value=" + String(data.pressure));
      toInflux(line);
      line = String(BoardId + ".bme680.gas value=" + String(data.gas_resistance));
      toInflux(line);
      


/*
		Serial.print(String(millis()) + ", ");
		Serial.print(String(data.temperature * 9/5 + 32) + ", ");
		Serial.print(String(data.pressure) + ", ");
		Serial.print(String(data.humidity) + ", ");
		Serial.print(String(data.gas_resistance) + ", ");
		Serial.println(data.status, HEX);
*/	
  
  }
}



void distToInflux(float distance)
{

      String line = String("tideman3." + BoardId + ".distance value=" + String(distance));
      toInflux(line);

}


void getDistance()
{

  if (Serial1.available()) {

    uint8_t c = Serial1.read();
    // Serial1.println(c, HEX);

    // See if this is a header byte
    if (idx == 0 && c == 0xFF) {
      buffer[idx++] = c;
    }
    // Two middle bytes can be anything
    else if ((idx == 1) || (idx == 2)) {
      buffer[idx++] = c;
    }
    else if (idx == 3) {
      uint8_t sum = 0;
      sum = buffer[0] + buffer[1] + buffer[2];
      if (sum == c) {
        distance = ((uint16_t)buffer[1] << 8) | buffer[2];
        newData = true;
      }
      idx = 0;
    }
  }
  
  if (newData) {

    //Serial.printf("getSonarDistance NEW DATA\n");

    mDistance[count] = distance;
    count++;
    
    //Serial.print("Distance: ");
    //Serial.print(distance/25.4/12);
    //Serial.println(" ft");
    
    if (count == 1000)
    {
      //distToInflux(distance/25.4/12);

      qsort(mDistance, count, sizeof(mDistance[0]), sort_desc);

      disthigh = mDistance[50]/25.4;
      distlow = mDistance[count-50]/25.4;

      avg = average(mDistance,count)/25.4;
      count = 0;
      
      WaterLevelHigh=SENSORHEIGHTINFEET-((distlow)/12.0);
      WaterLevelLow=SENSORHEIGHTINFEET-((disthigh)/12.0);
      WaterLevelAvg=SENSORHEIGHTINFEET-((avg)/12.0);

      // logall();


    }

  }
}


void logDistance()
{

      toInflux(BoardId + ".WaterLevelAvg value=" + String(WaterLevelAvg));
      toInflux(BoardId + ".WaterLevelHigh value=" + String(WaterLevelHigh));
      toInflux(BoardId + ".WaterLevelLow value=" + String(WaterLevelLow));
      toInflux(BoardId + ".DistanceAvg value=" + String(avg/12.0));


}




void setup() 
{

    Serial.begin(115200);

    FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);     

    pinMode(batteryPin, INPUT);   // READ BATTERY 

    

    //voltageReset();  //reset when needed.


    //WIFI SETUP  
    wifiSetup();

    //Setup BatteryPin
    pinMode(batteryPin, INPUT);


    //OTA Setup
    otaSetup();


    //setupBME680();  // BME680  temp-hum-press-air
    //setupLTR390();  //LTR390 light and UV
    //setupAHT20();  //setup TEMP sensor
    //setup Pressure Sensor
    //setupICP();

    //AS5600 Setup Magnet Sensor
    //setupAS5600();


    setupNVS();

   

  //works line = String("cpu_temp2,location=server_room,sensor_type=102.168.0.1 value=75") ;
  
  line = String(BoardId + ".wifi.ip" + ",ipaddress=" + WiFi.localIP().toString() + " value=75") ;

  toInflux(line);

      //DEEPSLEEP
    // deepSleeplog(60,120);  //LOG TIME, SLEEP TIME


}



void voltageReset() {
  writeNVSFloat("highestVoltage", 0);
  writeNVSFloat("lowestVoltage", 100);
}

void OTAloop(){

     //OTA
    webserver.handleClient();
    ElegantOTA.loop();
}

void loop() {


  OTAloop();

  int  elapsed = millis() - lastLoopTime;



  leds[0] = CRGB::Green;     //LED shows red light
  FastLED.show();

  if (elapsed > 3000){

    leds[0] = CRGB::Blue;     // LED shows blue light
    FastLED.show();

    //logTemperature();
    //logICPressure();

    //logBME680();
    //logLTR390();

    ////logBatteryLevel();
 
    logDistance();

    logWifiStatus();

    lastLoopTime = millis();
    
  }

}
