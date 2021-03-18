#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SparkFunBQ27441.h>
#include "SparkFun_SHTC3.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <PMTK.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define PWS 34
#define KEY 26
#define NS 13
#define VIO 12

const char *ssid = "HUAWEI-8e4e";
const char *password = "ifran123";

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 850; // e.g. 850mAh battery
SHTC3 mySHTC3;                             // Declare an instance of the SHTC3 class
//Your Domain name with URL path or IP address with path
String serverName = "http://velovolt.ddns.net:1880/moaad/";

int p;
String imei = "";
String ipAddress = "";
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
uint8_t noGsmCounter = 0;

String server = "";
String userName = "";
String passWord = "Isis2019";
String fileName = "";
String filePath = "";
String fileSize = "";
String version = "";

String Hum;
String Temp;
String soc;
String volts;
String current;
String capacity;
String fullCapacity;
String power;
String health;

/******************Variable GPS S39EA  ******************/
String gpsData;
String Heure = "";
String latitude = "";
String LatitudeDir = "";
String Longtitude = "";
String longitudeDir = "";
String Fix = "";
String numSatelitte = "";
bool stopgps;

SoftwareSerial mySerial(25, 15); // RX,TX

/****************** Function GPS S39EA *******************/

double convertToDecimalDegrees(const char *latLon, const char *direction);
void getgpspoint(bool stopgps);
void substringGpsdata();

/******************-------------------********************/
uint16_t httpTimeout = 20000;
char dataToPost[150] = "";

void powerUp();
void powerDown();
bool gsmCheck(uint16_t waitInterval);
uint8_t getGsmStat(uint16_t timeOut);
void flushSim();
bool sendAtCom(long timeout, char *atCom, char *Rep, char *Error, int nbRep);
bool gprsOn();
void getIp(uint16_t TimeOut);
void getImei();
bool httpGet();
void powerCycle();
void wifiPrint(String message);
bool httpGet();
bool fireHttpAction(long timeout, char *Commande, char *Rep, char *Error);
void ftpGet(String passWord);
bool httpPost(char *payload);
void updateHumTemStats();
void errorDecoder(SHTC3_Status_TypeDef);
void setupBQ27441(void);
void updateBatteryStats();




void AppendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);

  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.print(message))
  {

    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }

  file.close();
}
void INTappendFile(fs::FS &fs, const char *path, int o)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);

  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.print(o))
  {

    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }

  file.close();
}





int zineb(fs::FS &fs, const char *path)
{  
  
     File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file)
  {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len)
    {
      size_t toRead = len;
      if (toRead > 512)
      {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    return flen;
    file.close();
  }
  else
  {
    Serial.println("Failed to open file for reading");
  }

  file.close();
}



size_t  pointReadingFile(fs::FS &fs, const char *path,size_t frameSize )
{
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);

  if (!file)
  {
    Serial.println("Failed to open file for reading");
   
  }

  char _rx_buffer[200];
  size_t g=file.readBytes((char *)_rx_buffer, frameSize);
  return g;

  Serial.print("Read from file: ");
  while (file.available())
  {
    /*affiche ce qu'on a ecrit*/ Serial.write(file.read());
  }
  file.close();
}

void setup()
{

  Serial.begin(9600);
  while (!Serial)
  {
    Serial.println("serial is ot ok");
  }

  if (!SD.begin(5))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  // String IPWifi = WiFi.localIP().toString();
  wifiPrint("Wifi IP address is " + WiFi.localIP().toString());
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.begin();

  // Serial.println("Ready");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  pinMode(VIO, OUTPUT);
  digitalWrite(VIO, HIGH);
  pinMode(PWS, INPUT);
  pinMode(NS, INPUT);
  powerDown();
  delay(1000);
  powerUp();
  Serial2.begin(4800);
  gsmCheck(20000);

  if (gprsOn())
  {
    wifiPrint("GPRS ON");
  }
  getImei();
  getIp(1000);
  Wire.begin();
  errorDecoder(mySHTC3.begin()); // To start the sensor you must call "begin()", the default settings use Wire (default Arduino I2C port)
  delay(5000);                   // Give time to read the welcome message and device ID.
  setupBQ27441();

  mySerial.begin(9600);
  delay(2000);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  mySerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(2000);
}
void loop()
{
  memset(dataToPost, 0, sizeof dataToPost);
  SHTC3_Status_TypeDef result = mySHTC3.update(); // Call "update()" to command a measurement, wait for measurement to complete, and update the RH and T members of the object
  updateHumTemStats();                            // This function is used to print a nice little line of info to the serial port
  delay(1000);                                    // Delay for the data rate you want - note that measurements take ~10 ms so the fastest data rate is 100 Hz (when no delay is used)
  updateBatteryStats();
  delay(1000);
  stopgps = false;
  getgpspoint(stopgps);
  substringGpsdata();

  const char *aa = latitude.c_str();
  const char *bb = LatitudeDir.c_str();
  double lat = convertToDecimalDegrees(aa, bb);
  latitude = String(lat, 6);

  //Serial.println(lat);

  aa = Longtitude.c_str();
  bb = longitudeDir.c_str();
  double longit = convertToDecimalDegrees(aa, bb);
  Longtitude = String(longit, 6);

  //Serial.println(longit);
  //imei est déterminé une seule fois dans Setup
  //String fix="1";
  //String lat="33.6009080";
  String lon = "-7.483875000";
  String spd = "015.48";
  String sat = "08";
  String crs = "0204.5";
  String dh = "1615825921";
  String flg = "00";
  sprintf(dataToPost, "[{\"X\":\"%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\"}]", imei.c_str(), Fix.c_str(), latitude.c_str(), Longtitude.c_str(), spd.c_str(), numSatelitte.c_str(), crs.c_str(), Heure.c_str(), flg.c_str(), soc.c_str(), volts.c_str(), current.c_str(), capacity.c_str(), fullCapacity.c_str(), power.c_str(), health.c_str(), Hum.c_str(), Temp.c_str());
  wifiPrint(dataToPost);
  httpPost(dataToPost);
  /* print the size of the last un writed point*/
  /*lire le fichier depuis bytes 19*/  // pointReadingFile(SD, "/i.txt",p);
  // int m =httpZPost(pointReadingFile(SD, "/numero de point non envoyee",m));
  // httpZPost(pointReadingFile(SD, "/numero de point non envoyee",m));
  ArduinoOTA.handle();
}
void setupBQ27441(void)
{
  if (!lipo.begin())
  {
    wifiPrint("Error: Unable to communicate with BQ27441.");
    wifiPrint("Check wiring and try again.");
    wifiPrint("(Battery must be plugged into Battery Babysitter!)");
  }
  else
  {
    wifiPrint("Connected to BQ27441!");
    lipo.setCapacity(BATTERY_CAPACITY);
  }
}
void updateBatteryStats()
{
  if (!lipo.begin())
  {
    soc = "0";
    volts = "0";
    current = "0";
    capacity = "0";
    fullCapacity = "0";
    power = "0";
    health = "0";
  }
  else
  {
    soc = String(lipo.soc());
    volts = String(lipo.voltage());
    current = String(lipo.current(AVG));
    capacity = String(lipo.capacity(FULL));
    fullCapacity = String(lipo.capacity(REMAIN));
    power = String(lipo.power());
    health = String(lipo.soh());
  }
}
void updateHumTemStats()
{
  if (mySHTC3.lastStatus == SHTC3_Status_Nominal) // You can also assess the status of the last command by checking the ".lastStatus" member of the object
  {
    Hum = String(mySHTC3.toPercent());
    Temp = String(mySHTC3.toDegC());
  }
  else
  {
    wifiPrint("Update failed, error: ");
    errorDecoder(mySHTC3.lastStatus);
  }
}
void errorDecoder(SHTC3_Status_TypeDef message) // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
{
  switch (message)
  {
  case SHTC3_Status_Nominal:
    wifiPrint("Nominal");
    break;
  case SHTC3_Status_Error:
    wifiPrint("Error");
    Hum = "Error";
    Temp = "Error";
    break;
  case SHTC3_Status_CRC_Fail:
    wifiPrint("CRC Fail");
    Hum = "CRC Fail";
    Temp = "CRC Fail";
    break;
  default:
    wifiPrint("Unknown return code");
    Hum = "Unknown return code";
    Temp = "Unknown return code";
    break;
  }
}
void powerUp()
{
  if ((analogRead(PWS) < 200))
  {
    wifiPrint("Powering UP...");
    pinMode(KEY, OUTPUT); //PWR KEY
    digitalWrite(KEY, LOW);
    delay(1200);
    pinMode(KEY, INPUT_PULLUP);
    delay(400);
  }
}
void powerDown()
{
  if ((analogRead(PWS) > 200))
  {
    wifiPrint("Powering DOWN");
    pinMode(KEY, OUTPUT); //PWR KEY
    digitalWrite(KEY, LOW);
    delay(1200);
    pinMode(KEY, INPUT_PULLUP); // Turn On the module
    delay(400);
  }
}
void getImei()
{
  Serial2.setTimeout(1000);
  Serial2.println("AT+GSN");
  String tempGSM3 = Serial2.readString();
  String tempIMEI = tempGSM3;
  if (strstr(tempGSM3.c_str(), "OK"))
  {
    imei = strstr(tempIMEI.c_str(), "8");
    imei = imei.substring(0, 15);
  }
  else
  {
    imei = "869170031000000";
  }
  String reply = "imei is " + imei;
  wifiPrint(reply);
  // //Serial.print("imei=");//Serial.println(imei);
}
bool gsmCheck(uint16_t waitInterval)
{
  currentMillis = millis();
  previousMillis = millis();
  uint8_t gsmStatInt = getGsmStat(3000);
  while ((gsmStatInt != 1) && (gsmStatInt != 5) && ((currentMillis - previousMillis) <= waitInterval) && (analogRead(PWS) > 200))
  {
    gsmStatInt = getGsmStat(2000);
    String debug = "gsm status is " + String(gsmStatInt);
    wifiPrint(debug);
  }
  if (gsmStatInt == 1)
  {
    wifiPrint("GSM Connected");
    // //Serial.println("GSM Connected");
  }
  else
  {
    wifiPrint("GSM NOT Connected");
    // //Serial.println("GSM NOT Connected");
  }
  if (((currentMillis - previousMillis) <= waitInterval))
  {
    if (analogRead(PWS) > 200)
    {
      noGsmCounter = 0;
      return true;
    }
  }
  else
  {
    noGsmCounter++;
    if (noGsmCounter == 5)
    {
      powerCycle();
      noGsmCounter = 0;
    }
    return false;
  }
}
uint8_t getGsmStat(uint16_t timeOut)
{
  flushSim();
  Serial2.setTimeout(timeOut);
  Serial2.println("AT+CREG?");
  String tempGSM = Serial2.readString();
  int ind1 = tempGSM.indexOf(',');
  String gsmStat = tempGSM.substring(ind1 + 1, ind1 + 2);
  return gsmStat.toInt();
}
void getIp(uint16_t TimeOut)
{
  wifiPrint("Obtaining IP Address...");
  Serial2.setTimeout(TimeOut);
  Serial2.println("AT+CSTT");
  String tempGSM0 = Serial2.readString();
  Serial2.println("AT+CIICR");
  tempGSM0 = Serial2.readString();
  Serial2.println("AT+CIFSR");
  tempGSM0 = Serial2.readString();
  uint8_t ind1 = tempGSM0.indexOf('.');
  uint8_t ind2 = tempGSM0.indexOf('.', ind1 + 1);
  uint8_t ind3 = tempGSM0.indexOf('.', ind2 + 1);
  uint8_t ind4 = ind3 + 3;
  ipAddress = tempGSM0.substring(11, ind4);
  String reply = "GSM IP address is " + ipAddress;
  wifiPrint(reply);
  // //Serial.print("IP address = ");//Serial.println(ipAddress);
}
void flushSim()
{
  uint16_t timeoutlp = 0;
  while (timeoutlp++ < 40)
  {
    while (Serial2.available())
    {
      Serial2.read();
      timeoutlp = 0; // If char was received reset the timer
    }
    delay(1);
  }
}
bool gprsOn()
{
  wifiPrint("turning ON GPRS...");
  // //Serial.println("turning on GPRS...");
  sendAtCom(5000, "AT+CFUN=1", "OK", "ERROR", 5);                      //"AT+CFUN=1"
  sendAtCom(5000, "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", "OK", 5); //"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""
  if (sendAtCom(5000, "AT+SAPBR=1,1", "OK", "OK", 5))
  { //"AT+SAPBR=1,1"
    if (sendAtCom(5000, "AT+CIICR", "OK", "ERROR", 5))
    { //"AT+CIICR"
      if (sendAtCom(5000, "AT+CIFSR", ">", "ERROR", 5))
      { //"AT+CIFSR"
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}
bool sendAtCom(long timeout, char *atCom, char *Rep, char *Error, int nbRep)
{ // findUntil
  flushSim();
  Serial2.setTimeout(timeout);
  for (uint16_t a = 0; a < strlen(atCom); a++)
  {
    Serial2.print(atCom[a]);
  }
  Serial2.println("");
  int compteur = 0;
  while ((!Serial2.findUntil(Rep, Error)) && (compteur < nbRep))
  {
    flushSim();
    for (uint16_t a = 0; a < strlen(atCom); a++)
    {
      Serial2.print(atCom[a]);
    }
    Serial2.println("");
    compteur++;
    delay(50);
  }
  if (compteur <= nbRep)
  {
    return true;
  }
  else
  {
    return false;
  }
  Serial2.setTimeout(1000);
}
void powerCycle()
{
  pinMode(VIO, OUTPUT);    // VIO SWEEP
  digitalWrite(VIO, HIGH); //turn On the module
  pinMode(PWS, INPUT);     //sim Power Status
  powerDown();
  delay(3000);
  powerUp();
  delay(100);
}
void wifiPrint(String message)
{
  for (uint16_t i = 0; i < message.length(); i++)
  {
    if (message[i] == ' ')
    {
      message[i] = '-';
    }
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    String serverPath = serverName + message;

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      // //Serial.print("HTTP Response code: ");
      // //Serial.println(httpResponseCode);
      String payload = http.getString();
      // //Serial.println(payload);
    }
    else
    {
      // //Serial.print("Error code: ");
      // //Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  else
  {
    // //Serial.println("WiFi Disconnected");
  }
}
bool httpGet()
{
  wifiPrint("Getting ftp server info...");
  sendAtCom(2000, "AT+HTTPINIT", "OK", "ERROR", 5);
  sendAtCom(2000, "AT+HTTPPARA=\"CID\",1", "OK", "ERROR", 5);
  uint8_t size = 83 + 15 + sizeof(ipAddress);
  char LINKChar[size] = {0};
  sprintf(LINKChar, "AT+HTTPPARA=\"URL\",\"http://velovolt.ddns.net:1880/update/chariots?IP=%s&IMEI=%s&VER=5.4\"", ipAddress.c_str(), imei.c_str());
  sendAtCom(5000, LINKChar, "OK", "ERROR", 5);
  if (fireHttpAction(5000, "AT+HTTPACTION=", "OK", "ERROR"))
  {
    Serial2.print("AT+HTTPREAD");
    Serial2.println("");
    String data = Serial2.readString();

    StaticJsonDocument<250> doc;
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, data.substring(29));
    // Test if parsing succeeds.
    if (error)
    {
      wifiPrint("deserializeJson() failed: ");
      wifiPrint(String(error.f_str()));
    }
    String version1 = doc["version"];
    version = version1;
    wifiPrint("parsed version is " + version);
    String server1 = doc["ftpServer"];
    server = server1;
    wifiPrint("parsed server is " + server);
    String userName1 = doc["userName"];
    userName = userName1;
    wifiPrint("parsed userName is " + userName);
    String fileName1 = doc["fileName"];
    fileName = fileName1;
    wifiPrint("parsed fileName is " + fileName);
    String filePath1 = doc["filePath"];
    filePath = filePath1;
    wifiPrint("parsed filePath is " + filePath);
    String fileSize1 = doc["fileSize"];
    fileSize = fileSize1;
    wifiPrint("parsed fileSize is " + fileSize);

    flushSim();
    sendAtCom(5000, "AT+HTTPTERM", "OK", "ERROR", 5);
  }
  else
  {
    wifiPrint("http get NOT successful");
    sendAtCom(5000, "AT+HTTPTERM", "OK", "ERROR", 5);
  }
}
bool fireHttpAction(long timeout, char *Commande, char *Rep, char *Error)
{
  flushSim();
  Serial2.setTimeout(timeout);
  Serial2.print(Commande);
  Serial2.println(1, DEC);
  if (Serial2.findUntil(Rep, Error))
  {
    return true;
  }
  else
  {
    return false;
  }
  flushSim();
  Serial2.setTimeout(1000);
}
void ftpGet(String PassWord)
{
  String tempFtp = "";
  uint8_t size = 0;
  wifiPrint("getting data from ftp server...");
  Serial2.setTimeout(500);
  Serial2.println("AT+FTPCID=1");
  tempFtp = Serial2.readString();
  wifiPrint("AT+FTPCID=1 " + tempFtp);

  size = 13 + strlen(server.c_str());
  char atComServer[size] = {0};
  sprintf(atComServer, "AT+FTPSERV=\"%s\"", server.c_str());
  Serial2.println(atComServer);
  tempFtp = Serial2.readString();
  wifiPrint(tempFtp);

  size = 11 + strlen(userName.c_str());
  char atComUserName[size] = {0};
  sprintf(atComUserName, "AT+FTPUN=\"%s\"", userName.c_str());
  Serial2.println(atComUserName);
  tempFtp = Serial2.readString();
  wifiPrint(tempFtp);

  size = 11 + strlen(PassWord.c_str());
  char atComPassWord[size] = {0};
  sprintf(atComPassWord, "AT+FTPPW=\"%s\"", PassWord.c_str());
  Serial2.println(atComPassWord);
  tempFtp = Serial2.readString();
  wifiPrint(tempFtp);

  size = 16 + strlen(fileName.c_str());
  char atComFileName[size] = {0};
  sprintf(atComFileName, "AT+FTPGETNAME=\"%s\"", fileName.c_str());
  Serial2.println(atComFileName);
  tempFtp = Serial2.readString();
  wifiPrint(tempFtp);

  size = 16 + strlen(filePath.c_str());
  char atComFilePath[size] = {0};
  sprintf(atComFilePath, "AT+FTPGETPATH=\"%s\"", filePath.c_str());
  Serial2.println(atComFilePath);
  tempFtp = Serial2.readString();
  wifiPrint(tempFtp);

  Serial2.setTimeout(15000);

  Serial2.println("AT+FTPTYPE?");
  tempFtp = Serial2.readString();
  wifiPrint("AT+FTPTYPE=0 " + tempFtp);

  Serial2.println("AT+FTPGET=1");
  tempFtp = Serial2.readString();
  wifiPrint("AT+FTPGET=1 " + tempFtp);

  wifiPrint("fileSize is " + strlen(fileSize.c_str()));
  size = 12 + strlen(fileSize.c_str());
  char atComFileSize[size] = {0};
  sprintf(atComFileSize, "AT+FTPGET=2,%s", fileSize.c_str());
  Serial2.println(atComFileSize);
  // tempFtp = Serial2.readString();
  // uint8_t ind1 = tempFtp.indexOf(','); Serial.print("index1=");Serial.println(ind1);
  // uint32_t ind2 = tempFtp.indexOf(',', ind1 + 1);Serial.print("index2=");Serial.println(ind2);
  // Serial.print("tempFTP=");Serial.print(tempFtp.substring(ind2+strlen(fileSize.c_str())+3));
  // Serial.print("tempFTP=");Serial.print(tempFtp);
  String reply = "tempFTP is " + Serial2.readString();
  wifiPrint(reply);
}
bool httpPost(char *payload)
{
  bool OkToSend = true;
  if (sendAtCom(3000, "AT+HTTPINIT", "OK", "ERROR", 5))
  { //"AT+HTTPINIT"
    if (sendAtCom(3000, "AT+HTTPPARA=\"CID\",1", "OK", "ERROR", 5))
    { //"AT+HTTPPARA=\"CID\",1"
      if (sendAtCom(5000, "AT+HTTPPARA=\"URL\",\"http://velovolt.ddns.net:8080/datasnap/rest/Tdata/rep\"", "OK", "ERROR", 5))
      { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
        Serial2.setTimeout(10000);
        flushSim();
        Serial2.print("AT+HTTPDATA=");
        delay(100);
        // uint16_t Size = ((p2-p1) * (SizeRec + 1)) + ((p2-p1) * 8) - 1 + 2;
        uint16_t Size = strlen(payload);
        wifiPrint("strlen payload is " + String(strlen(payload)));
        Serial2.print(Size);
        Serial2.print(",");
        uint32_t maxTime = 30000;
        Serial2.println(maxTime);
        Serial2.findUntil("DOWNLOAD", "ERROR");
        Serial2.print(payload);
        // Serial.print("[");
        // for (uint16_t i = p1; i < p2 ; i++)
        // {
        //   for (uint16_t j = SizeRec * i; j < (SizeRec * (i + 1)) ; j++)
        //   {
        //     if (j == (i * SizeRec)) {Serial.print("{\"P\":\"");delay(1);}
        //     uint16_t test = fram.read8(j);
        //     if ((test==0) ){sprintf(Buffer, "%c", 120); //x
        //     }else{sprintf(Buffer, "%c", test);}
        //     Serial.write(Buffer);
        //     delay(1);
        //   }
        //   Serial.print("\"}");
        //   delay(1);
        //   if (i < p2 - 1) {Serial.write(",");delay(1);}
        // }
        // Serial.print("]");
        Serial2.findUntil("OK", "OK");
      }
      else
        OkToSend = false;
    }
    else
      OkToSend = false;
  }
  else
    OkToSend = false;
  if (OkToSend)
  {
    if (fireHttpAction(httpTimeout, "AT+HTTPACTION=", ",200,", "ERROR"))
    {
      sendAtCom(5000, "AT+HTTPTERM", "OK", "ERROR", 5);
      /*cree un fichier qui contient LES POINTS ENVOYEES*/
        AppendFile(SD, "/numero de point envoyee.txt", dataToPost);
        

      return true;
    }
    else
    {
      sendAtCom(5000, "AT+HTTPTERM", "OK", "ERROR", 5);
      wifiPrint("HTTP POST FAILED");
      AppendFile(SD, "/numero de point non envoyee.txt", dataToPost);
      /*calcule bytes de fichier*/int p=zineb(SD, "/numero de point non envoyee.txt"); 
      /*entree dans un fichier nameed as : ii*/ INTappendFile(SD, "/ii.txt", p);
                                                AppendFile(SD, "/ii.txt", ";");
                                                
       

      return false;
    }
  }
  else
  {
    return false;
  }
}

double convertToDecimalDegrees(const char *latLon, const char *direction)
{
  char deg[4] = {0};
  char *dot, *min;
  int len;
  double dec = 0;

  if ((dot = strchr(latLon, '.')))
  {                                   // decimal point was found
    min = dot - 2;                    // mark the start of minutes 2 chars back
    len = min - latLon;               // find the length of degrees
    strncpy(deg, latLon, len);        // copy the degree string to allow conversion to float
    dec = atof(deg) + atof(min) / 60; // convert to float
    if (strcmp(direction, "S") == 0 || strcmp(direction, "W") == 0)
      dec *= -1;
  }
  return dec;
}

void getgpspoint(bool stopgps)
{
  while (stopgps == false)
  {
    if (mySerial.available())
    {
      char c = mySerial.read();
      if (c == '$')
      {
        gpsData = mySerial.readStringUntil('$');
        //Serial.print(gpsData);
        stopgps = true;
      }
    }
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void substringGpsdata()
{
  Heure = getValue(gpsData, ',', 1);
  Serial.println(Heure);

  latitude = getValue(gpsData, ',', 2);
  Serial.println(latitude);

  LatitudeDir = getValue(gpsData, ',', 3);
  Serial.println(LatitudeDir);

  Longtitude = getValue(gpsData, ',', 4);
  Serial.println(Longtitude);

  longitudeDir = getValue(gpsData, ',', 5);
  Serial.println(longitudeDir);

  Fix = getValue(gpsData, ',', 6);
  Serial.println(Fix);

  numSatelitte = getValue(gpsData, ',', 7);
  Serial.println(numSatelitte);
}




// int httpZPost(size_t payload)
// {
//   bool OkToSend = true;
//   if (sendAtCom(3000, "AT+HTTPINIT", "OK", "ERROR", 5))
//   { //"AT+HTTPINIT"
//     if (sendAtCom(3000, "AT+HTTPPARA=\"CID\",1", "OK", "ERROR", 5))
//     { //"AT+HTTPPARA=\"CID\",1"
//       if (sendAtCom(5000, "AT+HTTPPARA=\"URL\",\"http://velovolt.ddns.net:8080/datasnap/rest/Tdata/rep\"", "OK", "ERROR", 5))
//       { //"AT+HTTPPARA=\"URL\",\"http://casa-interface.casabaia.ma/commandes.php\""
//         Serial2.setTimeout(10000);
//         flushSim();
//         Serial2.print("AT+HTTPDATA=");
//         delay(100);
//         // uint16_t Size = ((p2-p1) * (SizeRec + 1)) + ((p2-p1) * 8) - 1 + 2;
//         uint16_t Size = strlen(payload);
//         wifiPrint("strlen payload is " + String(strlen(payload)));
//         Serial2.print(Size);
//         Serial2.print(",");
//         uint32_t maxTime = 30000;
//         Serial2.println(maxTime);
//         Serial2.findUntil("DOWNLOAD", "ERROR");
//         Serial2.print(payload);
//         // Serial.print("[");
//         // for (uint16_t i = p1; i < p2 ; i++)
//         // {
//         //   for (uint16_t j = SizeRec * i; j < (SizeRec * (i + 1)) ; j++)
//         //   {
//         //     if (j == (i * SizeRec)) {Serial.print("{\"P\":\"");delay(1);}
//         //     uint16_t test = fram.read8(j);
//         //     if ((test==0) ){sprintf(Buffer, "%c", 120); //x
//         //     }else{sprintf(Buffer, "%c", test);}
//         //     Serial.write(Buffer);
//         //     delay(1);
//         //   }
//         //   Serial.print("\"}");
//         //   delay(1);
//         //   if (i < p2 - 1) {Serial.write(",");delay(1);}
//         // }
//         // Serial.print("]");
//         Serial2.findUntil("OK", "OK");
//       }
//       else
//         OkToSend = false;
//     }
//     else
//       OkToSend = false;
//   }
//   else
//     OkToSend = false;
//   if (OkToSend)
//   {
//     if (fireHttpAction(httpTimeout, "AT+HTTPACTION=", ",200,", "ERROR"))
//     {
//       sendAtCom(5000, "AT+HTTPTERM", "OK", "ERROR", 5);
//       /*cree un fichier qui contient LES POINTS ENVOYEES*/
//         AppendFile(SD, "/numero de point envoyee.txt", dataToPost);
        

//       //return true;
//     }
//     else
//     {
//       sendAtCom(5000, "AT+HTTPTERM", "OK", "ERROR", 5);
//       wifiPrint("HTTP POST FAILED");
//       AppendFile(SD, "/numero de point non envoyee.txt", dataToPost);
//       /*calcule bytes de fichier*/int p=zineb(SD, "/numero de point non envoyee.txt"); 
//       /*entree dans un fichier nameed as : ii*/ INTappendFile(SD, "/ii.txt", p);
//                                                 AppendFile(SD, "/ii.txt", ";");
//                                                 return p;
                                                
       

//       return 1;
//     }
//   }
//   else
//   {
//     return 0;
//   }
// }

