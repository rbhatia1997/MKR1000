// Code created & modified by Ronak Bhatia 
// https://www.linkedin.com/in/ronakbhatia/

// Citations for the code available on LinkedIn
// For any questions, commments, and concerns, please email rbhatia@g.hmc.edu


/*
 * Please note that you have to upload certificates for the HTTPS server that you are POST'ing data to.
 * Before using this code, you need to go to File -> Examples -> WiFi101 -> FirmwareUpdater and then 
 * run the code. Then, go to Tools -> WiFi101 Firmware Updater (it should be the one that allows you to 
 * select WINC1501 Model B 19.5.4) and hit add domain under "update SSL root certificates. Enter the server IP or 
 * address, attach the MKR1000, and then hit upload certificates to WiFi Module. Then, the code should work like normal. 
 * If this step isn't completed, it's highly likely that your code won't POST properly. 
 */


#include "DHT.h" // library for temp sensor
#include <ArduinoHttpClient.h>
#include <WiFi101.h>  // for WiFi and HTTP Client
#include <LiquidCrystal.h> // LCD interface 
#include <Adafruit_MLX90614.h> // for IR temperature sensor

#define DHTTYPE DHT11   // DHT 11
#define DHTPIN 6     // what digital pin we're connected to for the DHT sensor

#define trigPin1 14
#define echoPin1 13
#define trigPin2 10
#define echoPin2 9
#define trigPin3 8
#define echoPin3 7 // These pins are for the distance sensor to function, all digital pins. 

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // initializing the sensor 
DHT dht(DHTPIN, DHTTYPE); 


char ssid[] = "your_wifi_here"; 
char pass[] = "password"; // Internet Settings 

const char serverAddress[] = "insert_server_address_here";  // server you're sending your data to. Don't put the path here. 
int port = 80; // Put the port here. Default HTTP is 80. 


const int ledPin = LED_BUILTIN; //the led attached to the board
const int soundPin = A2;  // Pin that sound output is connected to 
int RedLedPin = A5;    // LED connected to analog pin 11
int GreenLedPin = A3;    // LED connected to analog pin 10
int BlueLedPin = A4;    // LED connected to analog pin 9
const int FSR_PIN = A0; // Pin connected to FSR/resistor divider
int fsrReading; // the analog reading from the FSR resistor divider
int LEDbrightness_R;
int LEDbrightness_G; 
int LEDbrightness_B; // for initializing the LED brightness variable. 
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 10000; // Measured resistance of 3.3k resistor
String response; // for server response 

LiquidCrystal lcd(0,1,2,3,4,5); // LCD set-up

byte smiley[8] = {
    B00000,
    B10001,
    B00000,
    B00000,
    B10001,
    B01110,
    B00000, 
    }; // coding a smiley face for the LCD display


long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor; // also for the distance sensor. 

int TempId_IR_1 = 035;  
int SoundId = 4330948551; 
int TempId_DHT_1 = 11; 
int DistId_Left = 041; 
int DistId_Middle = 042; 
int DistId_Right = 043; 
int PressId = 9375; 

// identification for the sensors 


String sensorType1 = "Temperature"; 
String sensorType2 = "Pressure"; 
String sensorType3 = "Sound";
String sensorType4 = "Distance"; // Types of sensor! 


WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port); // Setting up client

int status = WL_IDLE_STATUS;
int statusCode = 0; // initializing status code variable 


/* These are the status codes if they ever appear when running the code: 
 
 *  static const int HTTP_SUCCESS =0;

The end of the headers has been reached. This consumes the '\n.' Could not connect to the server.

static const int HTTP_ERROR_CONNECTION_FAILED =-1;

This call was made when the HttpClient class wasn't expecting it to be called. 
Usually indicates your code is using the class incorrectly.

static const int HTTP_ERROR_API =-2;

Spent too long waiting for a reply...

static const int HTTP_ERROR_TIMED_OUT =-3;

The response from the server is invalid, is it definitely an HTTP server?

static const int HTTP_ERROR_INVALID_RESPONSE =-4;

 */

// End of intialization of variables! 

void setup(){

  Serial.begin(9600); // initializing the sensors 
  lcd.createChar(0, smiley); 
  mlx.begin(); // starting the IR sensor
  lcd.begin(16, 2);
  
  pinMode(FSR_PIN, INPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  pinMode(ledPin,OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT); // setting the input/output pins

  while(!Serial);
    while ( status != WL_CONNECTED) {
      Serial.print("Attempting to connect to Network named: ");
      Serial.println(ssid);                   // print the network name (SSID);
      lcd.setCursor(0, 0);
      lcd.print(ssid); 
      lcd.setCursor(0, 1);
      lcd.print("Connecting..");
      delay(4000); 
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
    delay(4000); 
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
    analogWrite(GreenLedPin,250); // Turning light green 
    
    delay(4000);
    lcd.clear();
  }

  // end of Setup


void loop() {
  
  LEDbrightness_R = 0;
  LEDbrightness_G = 0;
  LEDbrightness_B = 0;
  delay(2000); 

  // Wait a few seconds between measurements.
   // Reading temperature or humidity takes about 250 milliseconds!
  // DHT11 readings may also be up to 2 seconds. It's fairly slow. 

  lcd.setCursor(0, 0);
  lcd.print("Snsrs On"); 
  lcd.setCursor(0, 1);
  lcd.print("POSTing...");
  delay(4000);
  lcd.clear();


// ============================== DHT11 TEMP Information Here! =================================

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
   float hic = dht.computeHeatIndex(t, h, false);

  Serial.println("Humidity: " + String(h) + " %\t. " + "Temperature: " + String(t) + " *C. " + String(f) + "*F\t. " +  "Heat index: " + String(hic) + " *C. " + String(hif) + " *F.");
  // You can also comment this out to minimize serial output.  

  /*
  Serial.println("");
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
  Serial.println("");
  Serial.println("");
  */ // Optional way of illustrating the data  


  if (f > 130 || h > 55){
   analogWrite(RedLedPin, 250);         
   analogWrite(GreenLedPin,0); // alert user through red LED
   lcd.setCursor(0, 0);
   lcd.print("HUMIDITY OR"); 
   lcd.setCursor(0, 1);
   lcd.print("TEMP WARNING!");
   delay(4000);
   lcd.clear();
  } 

  else {
   analogWrite(RedLedPin, 0);         
   analogWrite(GreenLedPin, 255);
  }
  
  delay(300); 

  // ============================== Sound Sensor Information Here! =================================


  int soundValue = analogRead(soundPin);
  String soundString = "Empty"; 
  
  if(soundValue > 1000) {
    
      Serial.println(""); 
      soundString = "Sound is too high! Please get to safety!"; 
      Serial.println("LOUD!!!");
      Serial.println(""); 

      analogWrite(RedLedPin, 250);         
      analogWrite(GreenLedPin,0);   

      lcd.setCursor(0, 0);
      lcd.print("Sound Loud");
      lcd.setCursor(0,1);
      lcd.print(soundValue);

      delay(4000);     
      lcd.clear();
      }

    else {
      
      Serial.println(""); 
      Serial.println("Normal Levels"); 
      Serial.println("");

      soundString = "Normal Levels"; 
      analogWrite(RedLedPin, 0);         
      analogWrite(GreenLedPin, 255);
      }
    delay(300);
    
  // ============================== FSR Sensor Information Here! =================================
  
    int fsrADC = analogRead(FSR_PIN);
    if (fsrADC != 0) // If the analog reading is non-zero
    {
      // Use ADC reading to calculate voltage:
      float fsrV = fsrADC * VCC / 1023.0;

      // Use voltage and static resistor value to 
      // calculate FSR resistance:
      float fsrR = R_DIV * (VCC / fsrV - 1.0);
      
      // Serial.println("");
      // Serial.println("Resistance: " + String(fsrR) + " ohms");
      // Serial.println(""); // if you want to know the resistance going through the FSR. 

      // the following calculations are based on the FSR's datasheet...

      float forceGrams;
      float fsrG = 1.0 / fsrR; // Calculate conductance

      if (fsrR <= 600) 
        forceGrams = (fsrG - 0.00075) / 0.00000032639;
      else
        forceGrams =  fsrG / 0.000000642857; // estimate of the parabolic curve by detaching into two linear curves. 

      // Serial.println("");
      // Serial.println("Force: " + String(forceGrams) + " g");
      // Serial.println(); // If you want to know the force. 

      if (fsrADC > 30){
        bool objectDetected = true; 
        analogWrite(RedLedPin, 0);         
        analogWrite(GreenLedPin, 255);
      } else {
         bool objectDetected = false; 
         lcd.setCursor(0, 0);
         lcd.print("NO OBJ DETECTED"); 
         lcd.setCursor(0, 1);
         lcd.print("SHOWING "+ String(forceGrams) +"g");
         
        analogWrite(RedLedPin, 250);         
        analogWrite(GreenLedPin,0);

        delay(4000);
        lcd.clear();
      }
      delay(300); 
    }
    
  // ============================== IR TEMP SENSOR INFORMATION HERE!  =================================

  /*
  Serial.println("");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
  Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  Serial.println("");
  */ 

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
  Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  Serial.println(""); 
  
  // The above, when uncommented, will display the IR Temperature readings to the Serial port. 

  if (mlx.readObjectTempF() > 130){

   Serial.println("");
   Serial.println("Too hot!"); 
   Serial.println("");

   analogWrite(RedLedPin, 250);         
   analogWrite(GreenLedPin,0);

   lcd.setCursor(0, 0);
   lcd.print("OBJ TOO HOT!"); 
   lcd.setCursor(0, 1);
   lcd.print("USE SLEEVE!");

   delay(4000); 
   lcd.clear();

   if (mlx.readObjectTempF() < 60){

     Serial.println("");
     Serial.println("Too cold or object spilled!"); 
     Serial.println("");

     analogWrite(RedLedPin, 250);         
     analogWrite(GreenLedPin,0);

     lcd.setCursor(0, 0);
     lcd.print("2 COLD OR SPILL!"); 
     lcd.setCursor(0, 1);

     lcd.print("PLS HELP ME!");

     delay(4000);
     lcd.clear();
   }
  }
  
  else {
   analogWrite(RedLedPin, 0);         
   analogWrite(GreenLedPin, 255);
  }
  
  delay(300);

  // ============================== DISTANCE SENSOR INFORMATION HERE!  =================================


    SonarSensor(trigPin1, echoPin1);
    RightSensor = distance;
    SonarSensor(trigPin2, echoPin2);
    LeftSensor = distance;
    SonarSensor(trigPin3, echoPin3);
    FrontSensor = distance;

    Serial.print(String(LeftSensor) + " - " + String(FrontSensor) + " - " + String(RightSensor));    
    
    /*
    Serial.println(""); 
    Serial.print(LeftSensor);
    Serial.print(" - ");
    Serial.print(FrontSensor);
    Serial.print(" - ");
    Serial.println(RightSensor);
    Serial.println(""); 
    */ 
    // Another way to represent the data ^^^ 

    if (LeftSensor > 10 && FrontSensor > 10 && RightSensor > 10){ // if all sensors are detecting no object in range... 

       analogWrite(RedLedPin, 250);         
       analogWrite(GreenLedPin,0);

       Serial.print(""); 
       Serial.println("There is no object detected..."); 
       Serial.print(""); 

        lcd.setCursor(0, 0);
        lcd.print("NO OBJ PRSNT!"); 
        lcd.setCursor(0, 1);
        lcd.print("CHECK FOR SPILL!");

        delay(4000); 
        lcd.clear();
    } 

    else {
      analogWrite(RedLedPin, 0);         
      analogWrite(GreenLedPin, 255);
    }
    delay(300); 

  // ============================== POST'ING INFORMATION HERE!  =================================

  String robotName = "Freight73";
  String contentType = "application/x-www-form-urlencoded"; //this can be application/json if using json. As mentioned in my article, I use XML encoded, but I would recommend JSON. 

  String postData= "temp=" + String(round(f)) + "&robotName=" + String(robotName); // I'm only showing the POST for one piece of data - the IR Temperature reading. 
  // for multiple sensors, you'd have to edit the backend server to recieve that. You'd do the same process (several posts) for the different sensors. MongoDB is a popular tool for backend. 

  // for this example, I only POST'd temperature data in XML encoded format. Future work would involve POST'ing all sensor data using Node.js.

  Serial.println(""); 
  Serial.println(postData);
  Serial.println(""); 
  Serial.println("making POST request");
  Serial.println(""); 

  String path = "/path/user_defined_pathforPOSTING"; // for example, if you have 129.32.32.3 (made-up) as your IP and your POST information was on 129.32.32.3/MKR1000/sensors then you'd put "/MKR1000/sensors" here. 

  lcd.setCursor(0, 0);
  lcd.print("Sensors On"); 
  lcd.setCursor(0, 1);
  lcd.print("POSTing...");

  delay(4000);
  lcd.clear();

  client.post(path, contentType, postData); // The command that actually does the POST! 

  // read the status code and body of the response
  
  int statusCode = client.responseStatusCode();
  String response = client.responseBody();


  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response); // HTTP responses.. 

  lcd.setCursor(0, 0);
  lcd.print(statusCode); 
  lcd.setCursor(0, 1);
  lcd.print(response);
  delay(3000); 
  lcd.clear(); 

  lcd.write(byte(0)); // outputs smiley face! 
  lcd.setCursor(0, 1);
  lcd.print("Waiting 10 sec"); 
  Serial.println(""); 
  Serial.println("Wait ten seconds\n");

  analogWrite(RedLedPin, 0);         
  analogWrite(GreenLedPin, 255);

  delay(10000); // every ten seconds you send sensor data. You can change this, but do so at your own risk. I wouldn't go below 2-3 seconds. 
  lcd.clear();
}


void SonarSensor(int trigPin,int echoPin) // helper function for the distance sensor! 
  {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  }


