// Code created & modified by Ronak Bhatia
// https://www.linkedin.com/in/ronakbhatia/

// Citations for the code available on LinkedIn
// For any questions, commments, and concerns, please email rbhatia@g.hmc.edu

// PLEASE FOLLOW INSTRUCTIONS HERE BEFORE USING THIS: http://arduino-tweet.appspot.com/

// This code takes data from temperature sensors (both LM35 and IR Temperature) and tweets if it crosses a certain threshold.
// This idea can be applied for use for multiple different sensors.

// WARNING BEFORE USE:
// The library uses the above site as a proxy server for OAuth stuff. Your tweet may not be applied during maintenance of this site.
// Please avoid sending more than 1 request per minute not to overload the server.
// Twitter seems to reject repeated tweets with the same content (returns error 403).


/*
   Please note that you have to upload certificates for the HTTPS server that you are POST'ing data to.
   Before using this code, you need to go to File -> Examples -> WiFi101 -> FirmwareUpdater and then
   run the code. Then, go to Tools -> WiFi101 Firmware Updater (it should be the one that allows you to
   select WINC1501 Model B 19.5.4) and hit add domain under "update SSL root certificates. Enter the server IP or
   address, attach the MKR1000, and then hit upload certificates to WiFi Module. Then, the code should work like normal.
   If this step isn't completed, it's highly likely that your code won't POST properly.
*/


#include <LiquidCrystal.h>    //arduino lcd library
#include <Adafruit_MLX90614.h>
#include <SPI.h>
#include <WiFi101.h>
#include <Twitter.h> // please make sure you have this library 

// Available for download here: https://playground.arduino.cc/Code/TwitterLibrary
// Follow directions here first: http://arduino-tweet.appspot.com/

#include <string.h>

void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}


Adafruit_MLX90614 mlx = Adafruit_MLX90614();


char ssid[] = "";     //  your network SSID (name)
char pass[] = "";  // your network password
int status = WL_IDLE_STATUS;     // the WiFi radio's status

Twitter twitter(""); // you're going to get a code from here. If you don't know what this means, please read the intro.
char combined[] = "Test"; // Initializing a variable for later use.

LiquidCrystal lcd(0, 1, 2, 3, 4, 5); //defining lcd pins

int value = 0;          //initializing variables
float volts = 0.0;
float temp = 0.0;
float tempF = 0.0;
int ledPin1 = 6;
int ledPin2 = 7;
int ledPin3 = 8;
long randNumber; // prevents twitter for getting mad at me for having similar sounding tweets.
String Location = "San Francisco, CA";


void setup()
{
  Serial.begin(9600);   // opens serial port, sets data rate to 9600 bps
  lcd.begin(16, 2);     // set up the LCD's number of columns and rows
  mlx.begin();
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWiFiData();
}



void loop()
{
  // NOTE, THIS IS USING THE LM35 TEMP SENSOR
  value = analogRead(A0);        //read from A0
  volts = (value / 1023.0) * 5; //conversion to volts
  temp = volts * 100.0;          //conversion to temp Celsius
  tempF = temp * (9 / 5) + 32.0;     //conversion to temp Fahrenheit
  Serial.println("Calibration Temperature is...");
  Serial.println(tempF);
  Serial.println("");

  //display temp to lcd

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());
  Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

  Serial.println(); // reading information from the IR temp sensor
  lcd.setCursor(0, 0);
  lcd.print("AMBIENT= ");
  lcd.print(mlx.readAmbientTempF());
  lcd.print(" F");
  lcd.setCursor(0, 1);
  lcd.print("OBJECT= ");
  lcd.print(mlx.readObjectTempF());
  lcd.print("  F");

  if (mlx.readObjectTempF() > 130 || mlx.readObjectTempF() < 70) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    char combined[280]; // want to keep the amount of characters below Twitter's maximum.
    randNumber = random(100000);
    String Combined_Data = "Hi! I'm currently in " + Location +  "!" + " Unfortunately, your coffee is too hot!!  " + "Error Code: " + String(randNumber); // change this for what you want it to tweet.
    // I had my IR temperature sensor hooked up to a coffee mug and it would tweet me to let me know if my coffee was too hot.
    Combined_Data.toCharArray(combined, 280);
    if (twitter.post(combined)) { // where the twitter magic happens
      // Specify &Serial to output received response to Serial.
      // If no output is required, you can just omit the argument, e.g.
      // int status = twitter.wait();
      int status = twitter.wait(&Serial);
      if (status == 200) {
        Serial.println("OK.");
      } else {
        Serial.print("failed : code ");
        Serial.println(status);
      }
    } else {
      Serial.println("connection failed.");
    }
  } else {
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
  }

  delay(500);  
}

