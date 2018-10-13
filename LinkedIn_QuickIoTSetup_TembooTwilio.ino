// This code involves two push buttons and the MKR1000. 
// It will send a text message to a specified end user when both buttons are pressed. Will work as long as WiFi is enabled.
// Acknowledging the Arduino pushbutton page + Arduino state switch page for resources. 
// Ronak Bhatia - HMC '19 


#include <SPI.h>
#include <WiFi101.h>
#include <WiFiSSLClient.h>
#include <TembooSSL.h>
#include "TembooAccount.h" // Contains Temboo account information

WiFiSSLClient client;

int calls = 1;   // Execution count, so this doesn't run forever
int maxCalls = 3;   // Maximum number of times the Choreo should be executed
const int buttonPin = 2;     // the number of the pushbutton pin
int inPin = 5;         // the number of the input pin
int outPin = LED_BUILTIN;       // the number of the output pin
const int ledPin =  LED_BUILTIN;      // the number of the LED pin
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;         // variable for reading the pushbutton status

int state = HIGH; // current reading from output
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers


void setup() {
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);

  // For debugging, wait until the serial console is connected
  delay(4000);

  int wifiStatus = WL_IDLE_STATUS;

  // In Temboo, make sure to say you're using an Arduino Zero + WiFi101.

  // Determine if the WiFi Shield is present
  Serial.print("\n\nShield:");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("FAIL");

    // If there's no WiFi shield, stop here
    while (true);
  }

  Serial.println("MKR1000 Detected");

  // Try to connect to the local WiFi network
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("WiFi:");
    wifiStatus = WiFi.begin(WIFI_SSID, WPA_PASSWORD);

    if (wifiStatus == WL_CONNECTED) {
      Serial.println("Operational!");
    } else {
      Serial.println("FAIL");
    }
    delay(5000);
  }

  Serial.println("Setup complete.\n");
}

void loop() {
  buttonState = digitalRead(buttonPin);
  reading = digitalRead(inPin);

  if (calls <= maxCalls) {
    if (reading == HIGH && previous == LOW && millis() - time > debounce) {
      if (buttonState == HIGH) {
        digitalWrite(ledPin, HIGH);

        Serial.println("Running SendSMS - Run #" + String(calls++));

        TembooChoreoSSL SendSMSChoreo(client);

        // Invoke the Temboo client
        SendSMSChoreo.begin();

        // Set Temboo account credentials
        SendSMSChoreo.setAccountName(TEMBOO_ACCOUNT);
        SendSMSChoreo.setAppKeyName(TEMBOO_APP_KEY_NAME);
        SendSMSChoreo.setAppKey(TEMBOO_APP_KEY);
        SendSMSChoreo.setDeviceType(TEMBOO_DEVICE_TYPE);

        // Set Choreo inputs
        String AuthTokenValue = "TODO: Insert Token Value";
        SendSMSChoreo.addInput("AuthToken", AuthTokenValue);
        String FromValue = "TODO: Insert value of Twilio Phone Number";
        SendSMSChoreo.addInput("From", FromValue);
        String ToValue = "TODO: Inset Number";
        SendSMSChoreo.addInput("To", ToValue);
        String BodyValue = "TODO: Insert Text Message";
        SendSMSChoreo.addInput("Body", BodyValue);
        String AccountSIDValue = "TODO: Insert Account SID";
        SendSMSChoreo.addInput("AccountSID", AccountSIDValue);

        // Identify the Choreo to run
        SendSMSChoreo.setChoreo("/Library/Twilio/SMSMessages/SendSMS");

        // Run the Choreo; when results are available, print them to serial
        SendSMSChoreo.run();

        while (SendSMSChoreo.available()) {
          char c = SendSMSChoreo.read();
          Serial.print(c);
        }
        SendSMSChoreo.close();
      }
    } else {

      // turn LED off:

      digitalWrite(ledPin, LOW);
    }
    if (state == HIGH)
      state = LOW;
    else
      state = HIGH;
    time = millis();
  }
  previous = reading;

  Serial.println("\nWaiting...\n");
  delay(500); // check frequently for input
}
