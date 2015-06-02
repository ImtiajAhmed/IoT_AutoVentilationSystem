
/*
  Imtiaj Ahmed & Oswald Barral 
  Dept. of Computer Science
  University of Helsinki
  May 2015
  Contact: Imtiaj.Ahmed@helsinki.fi
  
This sketch used to read the dht22 sensor's data (temperature and humidity) then rotates the servo shaft
to open the ventilation window of the green house according to the percieved sensor's data.
Then, these values upload to sparkfun server https://data.sparkfun.com/streams/bGgzxRJy5pTmNgbdd8py/update/Vp1AxrMKeyuoErz11y26



Some parts of this program has been picked from adafruit and sparkfun
Here we used adafruit cc3000 Wifi Breakout and its library to connect wifi
instead of using Sparkfun's wifi shield and its library.

****************************************************************
 an example for the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469
****************************************************************
Phant_CC3000.ino
Post data to SparkFun's data stream server system (phant) using
an Arduino and the CC3000 Shield.
Jim Lindblom @ SparkFun Electronics
Original Creation Date: July 3, 2014
(http://data.sparkfun.com)
https://learn.sparkfun.com/tutorials/pushing-data-to-datasparkfuncom/all#arduino--cc3000-shield
****************************************************************
*/

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include <Progmem.h>

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "**********"        // Use your own SSID
#define WLAN_PASS       "**********"        // Use your own password
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  30000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
char server[] = "data.sparkfun.com";      // Remote host site

/////////////////
// Phant Stuff //
/////////////////
const String publicKey = "*************"; // Use your own public key for the data.sparkfun.com
const String privateKey = "************"; // Use your own private key for the data.sparkfun.com
const byte NUM_FIELDS = 3;
const String fieldNames[NUM_FIELDS] = {"temperature", "humidity", "servoangle"};
String fieldData[NUM_FIELDS];

/////////////////
// dht22 Stuff //
/////////////////
#include <Servo.h> 
#include <dht.h>

dht DHT;
#define DHT22_PIN A5

int humidity = 0, temperature = 0;

Servo myservo; 
int servoMinVal = 0;
int lastServoVal = servoMinVal;    // variable to store the servo's last position value
int newServoVal = servoMinVal;    // variable to store the servo's new position value to move
int SERVO_PIN = 9;
int SERVO_MAX = 180;
unsigned long PERIOD = 600000; //10 minutes

uint32_t ip;

int RESTART_PIN = 7; //used to restart the system

void setup(void)
{
  Serial.begin(115200);
  delay(500);
  
  Serial.println("DHT_Status, \tHumidity (%),\tTemperature (C),\tTime (us), \tServoPos (degree)");

  
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    //while(1);
    SysRestart();
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    //while(1);
    SysRestart();
  }
   
  Serial.println(F("Connected!"));
  
  unsigned long endtime = millis() + IDLE_TIMEOUT_MS;
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    Serial.println(F("Trying to check DHCP!"));
    if(millis()>endtime)
    {
      Serial.println(F("DHCP time out"));
      SysRestart(); //restart the system
    }
    else delay(100); // ToDo: Insert a DHCP timeout!
  }  

  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  endtime = millis() + IDLE_TIMEOUT_MS;
  ip = 0;
  // Try looking up the website's IP address
  Serial.print(server); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(server, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    if(millis()>endtime)
    {
      Serial.println(F("DHCP time out"));
      SysRestart(); //restart the system
    }
    else delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  // Optional: Do a ping test on the website
 /* 
  Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");  
  replies = cc3000.ping(ip, 5);
  Serial.print(replies); Serial.println(F(" replies"));
  */  
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object 
  delay(15);
  myservo.write(servoMinVal);
  delay(15);
  myservo.detach();
  delay(2000);
}

void SysRestart()
{
  Serial.println(F("System Restarting....!"));
  delay(5000); //5 seconds
  pinMode(RESTART_PIN, OUTPUT);      // sets the restart pin as output
  digitalWrite(RESTART_PIN, LOW); //send low to reset pin to restart the system
}

void loop(void)
{
  if(UpdateDHTSensorData()==1) //if sensor returns ok then do the rest of the task
  {
    MapDHTtoServo();
    UpdateServoPosition();
    fieldData[0] = String(temperature);
    fieldData[1] = String(humidity);
    fieldData[2] = String(lastServoVal);
    // Post data:
    Serial.println("Posting!");
    postData(); // the postData() function does all the work, 
                 // check it out below. 
    delay(PERIOD);
  }
}


int UpdateDHTSensorData()
{
  uint32_t startM = micros();
  int check = DHT.read22(DHT22_PIN); // read sensor status
  uint32_t stopM = micros();
  
  switch (check)
    {
    case DHTLIB_OK:
        Serial.print("OK,\t");
        humidity = (int)DHT.humidity;
        temperature = (int)DHT.temperature;
        // DISPLAY DATA
        Serial.println();
        Serial.print(humidity);
        Serial.print(",\t");
        Serial.print(temperature);
        Serial.print(",\t");
        Serial.print(stopM - startM);
        Serial.print(",\t");
        return 1;
    case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Time out error,\t");
        break;
    case DHTLIB_ERROR_CONNECT:
        Serial.print("Connect error,\t");
        break;
    case DHTLIB_ERROR_ACK_L:
        Serial.print("Ack Low error,\t");
        break;
    case DHTLIB_ERROR_ACK_H:
        Serial.print("Ack High error,\t");
        break;
    default:
        Serial.print("Unknown error,\t");
        break;
    }
    return 0; //error occured
}

void MapDHTtoServo()
{
  //map Temperature to Servo position
  int temp2SrvPos = map((int)temperature, 25, 35, servoMinVal, SERVO_MAX+servoMinVal);     // scale it to use it with the servo (value between 0 and 90) 
  //map Humidity to Servo position
    int humi2SrvPos = map((int)humidity, 35, 50, servoMinVal, SERVO_MAX+servoMinVal);     // scale it to use it with the servo (value between 0 and 90) 
  
  newServoVal = temp2SrvPos > humi2SrvPos ? temp2SrvPos : humi2SrvPos;  
  if(newServoVal > (SERVO_MAX+servoMinVal)) newServoVal = SERVO_MAX+servoMinVal;
  else if(newServoVal < servoMinVal) newServoVal = servoMinVal;
}
//update servo position from old position to new position
void UpdateServoPosition()
{
  int pos;
 
  if(abs(lastServoVal - newServoVal)>5)
  {
    myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN
    delay(5);
    
    if(lastServoVal < newServoVal)
    {
      for(pos = lastServoVal; pos < newServoVal; pos += 1)  // goes from lastServoValo degrees to newVal degrees 
      {                                  // in steps of 1 degree 
        myservo.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }
    else
    {
      for(pos = lastServoVal; pos >= newServoVal; pos -= 1)  // goes from lastServoValo degrees to newVal degrees 
      {                                  // in steps of 1 degree 
        myservo.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }      
  
    lastServoVal = newServoVal;
    myservo.detach();
  }
  Serial.println(newServoVal);
  Serial.println(lastServoVal);
  Serial.println();
}

void postData()
{
   /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  Adafruit_CC3000_Client client = cc3000.connectTCP(ip, 80);
  if (client.connected()) {
    // Post the data! Request should look a little something like:
  // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&time=5201 HTTP/1.1\n
  // Host: data.sparkfun.com\n
  // Connection: close\n
  // \n
  client.print("GET /input/");
  client.print(publicKey);
  client.print("?private_key=");
  client.print(privateKey);
  for (int i=0; i<NUM_FIELDS; i++)
  {
    client.print("&");
    client.print(fieldNames[i]);
    client.print("=");
    client.print(fieldData[i]);
  }
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(server);
  client.println("Connection: close");
  client.println();
  
  unsigned long endtime = millis() + IDLE_TIMEOUT_MS;
  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      Serial.print(c);
    } 
    if(millis()>endtime)
    {
      Serial.println(F("Connection timeout"));
      client.close();  
      SysRestart(); //restart the system
    }    
  }
  Serial.println();
  } else {
    Serial.println(F("Connection failed"));    
    return;
  }

  Serial.println(F("-------------------------------------"));
 
  client.close();  
}

void DisconnectWifi()
{
  Serial.println(F("-------------------------------------"));
  
  /* You need to make sure to clean up after yourself or the CC3000 can freak out */
  /* the next time your try to connect ... */
  Serial.println(F("\n\nDisconnecting"));
  cc3000.disconnect();
}

/**************************************************************************/
/*!
    @brief  Begins an SSID scan and prints out all the visible networks
*/
/**************************************************************************/

void listSSIDResults(void)
{
  uint32_t index;
  uint8_t valid, rssi, sec;
  char ssidname[33]; 

  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
