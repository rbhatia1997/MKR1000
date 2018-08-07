/*
  This will post data to the Firebase iOS application whenever necessary.
  This version of the code currently detects temperature being too hot and POSTs to firebase every 1 minute if an object is consistently being detected as hot.


   Please note that you have to upload certificates for the HTTPS server that you are POST'ing data to
   before using this code. You need to go to File -> Examples -> WiFi101 -> FirmwareUpdater and then
   run the code. Then, go to Tools -> WiFi101 Firmware Updater (it should be the one that allows you to
   select WINC1501 Model B 19.5.4) and hit add domain under "update SSL root certificates. Enter the server IP or
   address, attach the MKR1000, and then hit upload certificates to WiFi Module. Then, the code should work like normal.


  // Created by Ronak Kumar Bhatia, Harvey Mudd College intern
  // Feel free to message me at bhatia.ronak@gmail.com for any further questions.

  // I did modify code from external and other sources. If you require references or citation, please email me.
*/

#include <ArduinoHttpClient.h>
#include <WiFi101.h>
#include <LiquidCrystal.h> // LCD interface 
#include <Adafruit_MLX90614.h>
#include "DHT.h" // library for temp sensor

#define trigPin1 14
#define echoPin1 13
#define trigPin2 10
#define echoPin2 9
#define trigPin3 8
#define echoPin3 7 // These pins are for the distance sensor to function, all digital pins. 

#define DHTTYPE DHT11   // DHT 11
#define DHTPIN 6     // what digital pin we're connected to


DHT dht(DHTPIN, DHTTYPE); // initalizing DHT object

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

/////// Wifi Settings ///////

char ssid[] = "";
char pass[] = ""; // Enter your Internet Settings

char serverAddress[] = "fcm.googleapis.com";  // server address for firebase
char serverAddress2[] = "";  // server address for POST'ing data to. Please fill this with a valid server address.
int port = 80;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);
HttpClient client_iot = HttpClient(wifi, serverAddress2, port);
int status = WL_IDLE_STATUS;

String response;
String robotName = "Freight73"; // In my demo, I used a robot which was called this.
int statusCode = 0;

String path = "/sensor";  // THIS IS THE PATH FOR THE SERVER YOU'RE POSTING TO
// Mine had www.serveraddress.com/sensor so I included the /sensor part here.

String contentType = "application/json"; // using JSON to communicate information.
String quote = "\"";
String comma = ",";
String colon = ":"; // environmental variables to help me post JSON data
int RedLedPin = A5;    // LED connected to analog pin 5
int BlueLedPin = A4;    // LED connected to analog pin 4
int GreenLedPin = A3;    // LED connected to analog pin 3
int LEDbrightness_R;
int LEDbrightness_G; // for initializing the LED brightness variable.
int LEDbrightness_B;
const int soundPin = A2;  // Pin that sound output is connected to
int fsrReading; // the analog reading from the FSR resistor divider
const int FSR_PIN = A0; // Pin connected to FSR/resistor divider
const int ledPin = LED_BUILTIN; //the led attached to the board
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 10000; // Measured resistance of 3.3k resistor
int fsrADC = analogRead(FSR_PIN); // for the FSR
int counter = 0;
int soundValue = analogRead(soundPin); // getting SPL (sound pressure levels) from microphone
String soundString = "Empty";
unsigned long previousMillis = 0;        // will store last time updated
const unsigned long interval = 6000;   // Checks every 10 seconds.
float fsrV = fsrADC * VCC / 1023.0;
float fsrR = R_DIV * (VCC / fsrV - 1.0);
float fsrG = 1.0 / fsrR; // Calculate conductance

long duration, distance, RightSensor, BackSensor, FrontSensor, LeftSensor; // also for the distance sensor.

int SoundId = 4330948551; // sensor identification
int TempId_DHT_1 = 11;
int TempId_IR_1 = 35;  // identification for the sensors
int DistId_Left = 041;
int DistId_Middle = 042;
int DistId_Right = 043;
int PressId = 9375;

String sensorType1 = "Temperature"; // Types of sensor!
String sensorType2 = "Pressure";
String sensorType3 = "Sound";
String sensorType4 = "Distance";
String objectDetected = "objectDetected"; // initializing FSR strings.


byte smiley[8] = {
  B00000,
  B10001,
  B00000,
  B00000,
  B10001,
  B01110,
  B00000,
}; // literally, a smiley face but encoded!

LiquidCrystal lcd(0, 1, 2, 3, 4, 5); // LCD set-up

void setup() {
  Serial.begin(9600);
  mlx.begin();
  lcd.createChar(0, smiley);
  lcd.begin(16, 2);
  pinMode(FSR_PIN, INPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT); // setting up the various inputs/outputs


  while (!Serial);
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);
    lcd.setCursor(0, 0);
    lcd.print(ssid);
    lcd.setCursor(0, 1);
    lcd.print("Connecting..");
    delay(2000);
    lcd.clear();

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  lcd.setCursor(0, 0);
  lcd.print("Connected!");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  delay(3000);
  lcd.clear();

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip); // displaying the Wifi information for the user
  lcd.setCursor(0, 0);
  lcd.print(ssid);
  lcd.setCursor(0, 1);
  lcd.print(ip);
  analogWrite(RedLedPin, 0);
  analogWrite(GreenLedPin, 250);
  delay(3000);
  lcd.clear();
}

void loop() {

  unsigned long currentMillis = millis();
  float forceGrams;
  LEDbrightness_R = 0;
  LEDbrightness_G = 0;
  LEDbrightness_B = 0; // initializing the LED variables.

  lcd.setCursor(0, 0);
  lcd.print("Sensors On");
  lcd.setCursor(0, 1);
  lcd.print("Getting Data");
  delay(2000);
  lcd.clear();  // Telling the users that the sensors are all on at the moment.

  // DHT 11 Sensor Calculations/Information
  float f = dht.readTemperature(true); // temp in F
  float h = dht.readHumidity(); // humidity
  float hif = dht.computeHeatIndex(f, h); // heat index

  if (isnan(h) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  if (f > 130 || h > 58) {
    analogWrite(RedLedPin, 250);
    analogWrite(GreenLedPin, 0);
    lcd.setCursor(0, 0);
    lcd.print("HUMIDITY OR");
    lcd.setCursor(0, 1);
    lcd.print("TEMP WARNING!");
    delay(2000);
    lcd.clear();
  }
  else {
    analogWrite(RedLedPin, 0);
    analogWrite(GreenLedPin, 255);
  }

  // Sound sensor information

  if (soundValue > 900)
  {
    soundString = "Sound is too high! Please get to safety!";
    Serial.println("LOUD!!!");
    Serial.println("");
    analogWrite(RedLedPin, 250);
    analogWrite(GreenLedPin, 0);
    lcd.setCursor(0, 0);
    lcd.print("Sound Loud");
    lcd.setCursor(0, 1);
    lcd.print(soundValue);
    delay(2000);
    lcd.clear();
  }
  else
  {
    // Serial.println("Normal Levels");
    Serial.println("");
    soundString = "Normal Levels";
    analogWrite(RedLedPin, 0);
    analogWrite(GreenLedPin, 255);
  }

  // Pressure (FSR) Information

  if (fsrADC != 0) {
    if (fsrR <= 600)
      forceGrams = (fsrG - 0.00075) / 0.00000032639;
    // this information is obtained by approximating the parabolic curve as two linear lines
    // the parabolic curve is used to convert analog data to force in grams
    else
      forceGrams =  fsrG / 0.000000642857;

    if (forceGrams > 7) {
      analogWrite(RedLedPin, 0);
      analogWrite(GreenLedPin, 255);
      lcd.setCursor(0, 0);
      lcd.print("Object Present");
      lcd.setCursor(0, 1);
      lcd.print(String(forceGrams) + " grams on.");
      delay(4000);
      lcd.clear();
    } else {
      analogWrite(RedLedPin, 0);
      analogWrite(GreenLedPin, 255);
      String objectDetected = "No Object Detected";
      lcd.setCursor(0, 0);
      lcd.print("No object!");
      lcd.setCursor(0, 1);
      lcd.print(String(forceGrams) + " grams prsnt");
      analogWrite(RedLedPin, 250);
      analogWrite(GreenLedPin, 0);
      delay(5000);
      lcd.clear();
    }
  }

  // Distance Sensor Information

  SonarSensor(trigPin1, echoPin1);
  RightSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  LeftSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  FrontSensor = distance;
  float averageDist = (LeftSensor + FrontSensor + RightSensor) / 3;

  if (LeftSensor > 10 && FrontSensor > 10 && RightSensor > 10) {
    analogWrite(RedLedPin, 250);
    analogWrite(GreenLedPin, 0);
    Serial.println("There is no object detected...");
    Serial.print("");
    lcd.setCursor(0, 0);
    lcd.print("NO OBJ PRESENT");
    lcd.setCursor(0, 1);
    lcd.print("POSSIBLE SPILL!");
    delay(2000);
    lcd.clear();
  }
  else {
    analogWrite(RedLedPin, 0);
    analogWrite(GreenLedPin, 255);
  }

  // IR Temperature sensor

  if (mlx.readObjectTempF() > 130) {
    Serial.println("");
    Serial.println("Too hot!");
    Serial.println("");
    analogWrite(RedLedPin, 250);
    analogWrite(GreenLedPin, 0);
    lcd.setCursor(0, 0);
    lcd.print("OBJ TOO HOT!");
    lcd.setCursor(0, 1);
    lcd.print("USE SLEEVE!");
    delay(3000);
    lcd.clear();

    // formatted POST data in JSON format. 
    // https://jsonlint.com/ is a fantastic site to check if your JSON output from Arduino is valid. 

    String postData1 = "{\"sensorId\":" + quote + String(TempId_IR_1) + quote + comma + "\"sensorType\":" + quote + String(sensorType1) + quote + comma + "\"sensorData\":" + "{" + quote + "temperatureFahrenheit" + quote + colon + String(mlx.readObjectTempF()) + "}" + "}";
    String postData2 = "{\"sensorId\":" + quote + String(SoundId) + quote + comma + "\"sensorType\":" + quote + String(sensorType3) + quote + comma + "\"sensorData\":" + "{" + quote + "Level" + quote + colon + quote + String(soundString) + quote + "}" + "}";
    String postData3 = "{\"sensorId\":" + quote + String(PressId) + quote + comma + "\"sensorType\":" + quote + String(sensorType2) + quote + comma + "\"sensorData\":" + "{" + quote + "forceGrams" + quote + colon + String(forceGrams) + comma + quote + "objectDetected" + quote + colon + quote + String(objectDetected) + quote + "}" + "}";
    String postData4 = "{\"sensorId\":" + quote + String(DistId_Middle) + quote + comma + "\"sensorType\":" + quote + String(sensorType4) + quote + comma + "\"sensorData\":" + "{" + quote + "distanceCentimeters" + quote + colon + quote + String(averageDist) + quote + "}" + "}";
    String postData5 = "{\"sensorId\":" + quote + String(TempId_DHT_1) + quote + comma + "\"sensorType\":" + quote + String(sensorType1) + quote + comma + "\"sensorData\":" + "{" + quote + "temperatureFahrenheit" + quote + colon + String(f) + "}" + "}";

    // POSTING IR TEMP DATA

    Serial.println(postData1);
    client_iot.post(path, contentType, postData1); // I called the server where I'm posting data to client_iot 

    // read the status code and body of the response

    int statusCode = client_iot.responseStatusCode();
    String response = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);

    lcd.setCursor(0, 0);
    lcd.print("POSTED IR TEMP");
    lcd.setCursor(0, 1);
    lcd.print("POSTING SOUND");
    delay(2000);
    lcd.clear();
    client.stop();

    // POSTING SOUND DATA

    Serial.println(postData2);
    client_iot.post(path, contentType, postData2);

    // read the status code and body of the response

    int statusCode2 = client_iot.responseStatusCode();
    String response2 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode2);
    Serial.print("Response: ");
    Serial.println(response2);

    lcd.setCursor(0, 0);
    lcd.print("POSTED SOUND");
    lcd.setCursor(0, 1);
    lcd.print("POSTING PRESSURE");
    delay(2000);
    lcd.clear();
    client.stop();

    // POSTING PRESSURE DATA

    Serial.println(postData3);
    client_iot.post(path, contentType, postData3);

    // read the status code and body of the response

    int statusCode3 = client_iot.responseStatusCode();
    String response3 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode3);
    Serial.print("Response: ");
    Serial.println(response3);

    lcd.setCursor(0, 0);
    lcd.print("POSTED PRESSURE");
    lcd.setCursor(0, 1);
    lcd.print("POSTING DISTANCE");
    delay(2000);
    lcd.clear();
    client.stop();

    // POSTING DISTANCE DATA

    Serial.println(postData4);
    client_iot.post(path, contentType, postData4);

    // read the status code and body of the response

    int statusCode4 = client_iot.responseStatusCode();
    String response4 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode4);
    Serial.print("Response: ");
    Serial.println(response4);

    lcd.setCursor(0, 0);
    lcd.print("POSTED DISTANCE");
    lcd.setCursor(0, 1);
    lcd.print("POSTING DHT11");
    delay(2000);
    lcd.clear();
    client.stop();

    // POSTING DHT11 DATA (AMBIENT TEMP DATA)

    Serial.println(postData5);
    client_iot.post(path, contentType, postData5);

    // read the status code and body of the response

    int statusCode5 = client_iot.responseStatusCode();
    String response5 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode5);
    Serial.print("Response: ");
    Serial.println(response5);

    lcd.setCursor(0, 0);
    lcd.print("POSTED DHT11");
    lcd.setCursor(0, 1);
    lcd.print("DONE POSTING!");
    delay(2000);
    lcd.clear();
    client.stop();

    /*

        This is an optional code that will allow for POST'ing when the temperature sensor data is too cold/possible spill is detected.

        if (mlx.readObjectTempF() < 60) {
          Serial.println("");
          Serial.println("Too cold!");
          Serial.println("");
          analogWrite(RedLedPin, 250);
          analogWrite(GreenLedPin, 0);
          lcd.setCursor(0, 0);
          lcd.print("POSSIBLE SPILL!");
          lcd.setCursor(0, 1);
          lcd.print("ALERT HUMAN!");
          delay(6000);
          lcd.clear();

          String postData3 = "{\"to\":" + quote + String(topic_path) + quote + comma + "\"priority\":" + quote + "high" + quote + comma + "\"notification\":" + "{" + quote + "body" + quote + colon + quote + "The object on the robot is currently too cold or there's a spill!" + quote + comma + quote + "title" + quote + colon + quote + "TEMPERATURE/SPILL ALERT!" + quote + "}" + "}";
          Serial.println(postData);

          // if (mlx.readObjectTempF() > 70 || mlx.readObjectTempF() < 60){
          client.beginRequest();
          client.post("/fcm/send");
          client.sendHeader("Content-Type", "application/json");
          client.sendHeader("Content-Length", postData3.length());
          client.sendHeader("Authorization", "key=AIzaSyBrVNs0Ovi3u8C4tz9OsD-hiplIrLkCAQg");
          client.beginBody();
          client.print(postData3);
          client.endRequest();

          // read the status code and body of the response
          statusCode = client.responseStatusCode();
          response = client.responseBody();

          Serial.print("Status code: ");
          Serial.println(statusCode);
          Serial.print("Response: ");
          Serial.println(response);
          client.endRequest();

          Serial.println("Wait five seconds");
          delay(5000);
        }

    */

    if (currentMillis - previousMillis >= interval)
    // the thought behind this part of the code was to prevent the supervisor from getting constant pings from the MKR1000 
    // Thus, I use a FSM (basically, fancy way of saying "smart" delay) to prevent the pings from happening more than every 1 minute. 
    {
      previousMillis += interval;
      analogWrite(RedLedPin, 250);
      analogWrite(GreenLedPin, 0);
      Serial.println("making POST request");
      String topic_path = ""; // this is the PATH FOR FIREBASE. So if you have fcm.googleapis.com/sensor/datatransfer, this would be "/sensor/datatransfer"
      String auth = "key="; // here is the authorization key for your firebase application. Please enter it. 
      Serial.println(auth);
      String postData = "{\"to\":" + quote + String(topic_path) + quote + comma + "\"priority\":" + quote + "high" + quote + comma + "\"notification\":" + "{" + quote + "body" + quote + colon + quote + "The object on the robot is currently too hot!" + quote + comma + quote + "title" + quote + colon + quote + "TEMPERATURE ALERT!" + quote + "}" + "}";
      Serial.println(postData);

      // so this is actually another way that you can do POST requests using the MKR1000
      // This is especially important when you need to add customer headers to your requests (in my case, an Authorization factor)

      client.beginRequest();
      client.post("/fcm/send");
      client.sendHeader("Content-Type", "application/json");
      client.sendHeader("Content-Length", postData.length());
      client.sendHeader("Authorization", "key="); // enter the key here again 
      client.beginBody();
      client.print(postData);
      client.endRequest();

      // read the status code and body of the response
      statusCode = client.responseStatusCode();
      response = client.responseBody();

      Serial.print("Status code: ");
      Serial.println(statusCode);
      Serial.print("Response: ");
      Serial.println(response);
      client.endRequest();

      lcd.setCursor(0, 0);
      lcd.print("ALERT SUPERVISOR");
      lcd.setCursor(0, 1);
      lcd.print("SENT TO IPAD");
      delay(4000);
      lcd.clear();
    }
  }

  else {
    // General POSTING of all sensor data to Allison's Servers.
    analogWrite(RedLedPin, 0);
    analogWrite(GreenLedPin, 255);

    String postData1 = "{\"sensorId\":" + quote + String(TempId_IR_1) + quote + comma + "\"sensorType\":" + quote + String(sensorType1) + quote + comma + "\"sensorData\":" + "{" + quote + "temperatureFahrenheit" + quote + colon + String(mlx.readObjectTempF()) + "}" + "}";
    String postData2 = "{\"sensorId\":" + quote + String(SoundId) + quote + comma + "\"sensorType\":" + quote + String(sensorType3) + quote + comma + "\"sensorData\":" + "{" + quote + "Level" + quote + colon + quote + String(soundString) + quote + "}" + "}";
    String postData3 = "{\"sensorId\":" + quote + String(PressId) + quote + comma + "\"sensorType\":" + quote + String(sensorType2) + quote + comma + "\"sensorData\":" + "{" + quote + "forceGrams" + quote + colon + String(forceGrams) + comma + quote + "objectDetected" + quote + colon + quote + String(objectDetected) + quote + "}" + "}";
    String postData4 = "{\"sensorId\":" + quote + String(DistId_Middle) + quote + comma + "\"sensorType\":" + quote + String(sensorType4) + quote + comma + "\"sensorData\":" + "{" + quote + "distanceCentimeters" + quote + colon + quote + String(averageDist) + quote + "}" + "}";
    String postData5 = "{\"sensorId\":" + quote + String(TempId_DHT_1) + quote + comma + "\"sensorType\":" + quote + String(sensorType1) + quote + comma + "\"sensorData\":" + "{" + quote + "temperatureFahrenheit" + quote + colon + String(f) + "}" + "}";

    // POSTING IR TEMP DATA

    Serial.println(postData1);
    client_iot.post(path, contentType, postData1);

    // read the status code and body of the response

    int statusCode = client_iot.responseStatusCode();
    String response = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);

    lcd.setCursor(0, 0);
    lcd.print("POSTED IR TEMP");
    lcd.setCursor(0, 1);
    lcd.print("POSTING SOUND");
    delay(2000);
    lcd.clear();
    client.stop();



    // POSTING SOUND DATA

    Serial.println(postData2);
    client_iot.post(path, contentType, postData2);

    // read the status code and body of the response

    int statusCode2 = client_iot.responseStatusCode();
    String response2 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode2);
    Serial.print("Response: ");
    Serial.println(response2);

    lcd.setCursor(0, 0);
    lcd.print("POSTED SOUND");
    lcd.setCursor(0, 1);
    lcd.print("POSTING PRESSURE");
    delay(2000);
    lcd.clear();
    client.stop();



    // POSTING PRESSURE DATA

    Serial.println(postData3);
    client_iot.post(path, contentType, postData3);

    // read the status code and body of the response

    int statusCode3 = client_iot.responseStatusCode();
    String response3 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode3);
    Serial.print("Response: ");
    Serial.println(response3);

    lcd.setCursor(0, 0);
    lcd.print("POSTED PRESSURE");
    lcd.setCursor(0, 1);
    lcd.print("POSTING DISTANCE");
    delay(2000);
    lcd.clear();
    client.stop();

    // POSTING DISTANCE DATA

    Serial.println(postData4);
    client_iot.post(path, contentType, postData4);

    // read the status code and body of the response

    int statusCode4 = client_iot.responseStatusCode();
    String response4 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode4);
    Serial.print("Response: ");
    Serial.println(response4);

    lcd.setCursor(0, 0);
    lcd.print("POSTED DISTANCE");
    lcd.setCursor(0, 1);
    lcd.print("POSTING DHT11");
    delay(2000);
    lcd.clear();
    client.stop();

    // POSTING DHT11 DATA (AMBIENT TEMP DATA)

    Serial.println(postData5);
    client_iot.post(path, contentType, postData5);

    // read the status code and body of the response

    int statusCode5 = client_iot.responseStatusCode();
    String response5 = client_iot.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode5);
    Serial.print("Response: ");
    Serial.println(response5);

    lcd.setCursor(0, 0);
    lcd.print("POSTED DHT11");
    lcd.setCursor(0, 1);
    lcd.print("DONE POSTING!");
    delay(2000);
    lcd.clear();
    client.stop();

    lcd.write(byte(0)); // outputs smiley face!
    lcd.setCursor(0, 1);
    lcd.print("Waiting 10 sec");
    Serial.println("");
    Serial.println("Restarting process\n"); // doing all the sensor work every 10 seconds. 

    analogWrite(RedLedPin, 0);
    analogWrite(GreenLedPin, 255);
    delay(2000);
    lcd.clear();
  }
}

void SonarSensor(int trigPin, int echoPin) // helper function for the distance sensor!
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
}

