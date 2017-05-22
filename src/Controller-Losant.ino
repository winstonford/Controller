/////////////////////////////////////////////////////////////////////////
//
// Canopy Controller
//
// Version 1.0
// Emmanuel Avionics, Inc. Los Angeles Jan 10, 2017
//
// Files needed:
// x.ino
// lib1.cpp
// lib1.h
// Compiles on the web based IDE at https://build.particle.io
// Runs on the Particle Photon board
//
////////////////////////////////////////////////////////////////////////////
//
#include "Particle.h"
#include "MQTT.h"
#include "SparkJson.h"
#include "elapsedMillis.h"
<<<<<<< HEAD
=======
#include "google-maps-device-locator.h"

>>>>>>> master
#include "lib1.h"

PRODUCT_ID(3104);
PRODUCT_VERSION(2);

#define APP_NAME "CanopyController"
#define FIRMWARE_VERSION "0.25"


SDL_Arduino_INA3221 ina3221;

// If defined, output directly to the serial port
#define DEBUG_SERIAL

// Time between samples in miliseconds
#define SAMPLETIME 1000

// INA3221 channels
#define CHAN1 1
#define CHAN2 2
#define CHAN3 3

// INA3221 boards (each board is hardware modified to define its channel)
#define BOARD0 0
#define BOARD1 1
#define BOARD2 2

// ADC channels for temperature measurement
#define ADC0 0
#define ADC1 1
#define ADC2 2

// INA3221 boards are numbered 0 thru 2
uint8_t board = 2;

// Loopcount is displayed so display always changes between frames
uint16_t loopcount;

// Constant to convert ADC counts to degrees Kelvin
const float ktemp = 100.0 * ((20.0 + 5.62) / 20.0) * (3.3 / 4095.0);

// replace these values with your own
/*
Losant requires the client ID, username, and password fields be correctly
set on all MQTT connect calls. client id must be set to a valid device ID
that is already registered with the Losant platform. username must be set
to a Losant access key. password must be set to a Losant access secret.
Access keys can be obtained through your application settings.
source: https://docs.losant.com/mqtt/overview/
*/
#define LOSANT_DEVICE_ID "58a1d1a603c8370001bf84c5"
#define LOSANT_ACCESS_KEY "7e488b34-723a-4c66-99b9-0e52e7eb2909"
#define LOSANT_ACCESS_SECRET "99296a5010261f96a051e943233759751092cbc21e58aba96d9fb24b8a7aa33f"

// Topic used to publish state to Losant.
String MQTT_TOPIC_STATE =
    String::format("losant/%s/state", LOSANT_DEVICE_ID);

// Topic used to subscribe to Losant commands.
String MQTT_TOPIC_COMMAND =
    String::format("losant/%s/command", LOSANT_DEVICE_ID);

void mqttCallback(char *topic, byte *payload, unsigned int length);

// ip of broker.losant.com: 104.197.8.180
byte mqttServer[] = {104, 197, 8, 180};
MQTT mqttClient(mqttServer, 1883, mqttCallback);

// check mqtt connection every 5 seconds
#define MQTT_RECONNECT_INTERVAL 5000
elapsedMillis mqttReconnectTimer;


/*******************************************************************************
 structure for writing thresholds in eeprom
 https://docs.particle.io/reference/firmware/photon/#eeprom
*******************************************************************************/
//randomly chosen value here. The only thing that matters is that it's not 255
// since 255 is the default value for uninitialized eeprom
// value 141 will be used in version 1.0
// this value HAS TO BE CHANGED every time the struct EepromMemoryStructure is changed
#define EEPROM_VERSION 141
#define EEPROM_ADDRESS 0

struct EepromMemoryStructure
{
  uint8_t version = EEPROM_VERSION;
  int outputD2;
  int outputD3;
  int outputD4;
  int outputD5;
  int outputD6;
};
EepromMemoryStructure eepromMemory;

// Global vars to store pin status
int outputD2 = LOW;
int outputD3 = LOW;
int outputD4 = LOW;
int outputD5 = LOW;
int outputD6 = LOW;

// google maps class
GoogleMapsDeviceLocator locator;

void setup(void)
{

  //publish startup message with firmware version
  Particle.publish(APP_NAME, "Firmware Version " FIRMWARE_VERSION, 60, PRIVATE);


  Serial.begin(57600);
  Serial.println("SDA_Arduino_INA3221_Test");

  Serial.println("Measuring voltage and current with ina3221 ...");
  // Initialize registers on each board
  ina3221.begin(0);
  ina3221.begin(1);
  ina3221.begin(2);
  // Initialize FET drivers
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);

  mqttConnect();

  // Set up internet switch control function
  Particle.function("switches", switchToggle);

//DEBU
  Particle.function("D2", toggleD2);

  //restore settings from eeprom, if there were any saved before
  readFromEeprom();
  //then update outputs accordingly
  updateOutputs();

  // google maps callback
  // Scan for visible networks and publish to the cloud every 30 seconds
  // Pass the returned location to be handled by the locationCallback() method
  // change the 30 seconds here to publish location as needed
  locator.withSubscribe(locationCallback).withLocatePeriodic(30);
  
}

// google maps callback declaration
void locationCallback(float lat, float lon, float accuracy) {
  // Handle the returned location data for the device. This method is passed three arguments:
  // - Latitude
  // - Longitude
  // - Accuracy of estimated location (in meters)
}

void loop(void)
{

  // google maps loop function call
  locator.loop();

#ifdef DEBUG_SERIAL
  Serial.print(loopcount++);
  Serial.println("------------------------------");
#endif
  // The boards are numbered 0 thru 3, the channels on each board 1 thru 3
  // Voltage V is in volts, current I is in milliamps
  float V1 = 0;
  float V2 = 0;
  float V3 = 0;
  float V4 = 0;
  float V5 = 0;
  float V6 = 0;
  float V7 = 0;
  float V8 = 0;
  float V9 = 0;
  float I1 = 0;
  float I2 = 0;
  float I3 = 0;
  float I4 = 0;
  float I5 = 0;
  float I6 = 0;
  float I7 = 0;
  float I8 = 0;
  float I9 = 0;
  float T1 = 0;
  float T2 = 0;
  float T3 = 0;
  // String for holding the data output ready to publish
  String outstring;

  int i;

  // Turn on power Zener Diode circuit
  myDigitalWrite(D6, HIGH);
  // Allow current to stabilize
  delay(50);

  // Get voltages
  V1 = ina3221.getBusVoltage_V(BOARD0, CHAN1);
  V2 = ina3221.getBusVoltage_V(BOARD0, CHAN2);
  V3 = ina3221.getBusVoltage_V(BOARD0, CHAN3);
  V4 = ina3221.getBusVoltage_V(BOARD1, CHAN1);
  V5 = ina3221.getBusVoltage_V(BOARD1, CHAN2);
  V6 = ina3221.getBusVoltage_V(BOARD1, CHAN3);
  V7 = ina3221.getBusVoltage_V(BOARD2, CHAN1);
  V8 = ina3221.getBusVoltage_V(BOARD2, CHAN2);
  V9 = ina3221.getBusVoltage_V(BOARD2, CHAN3);

  // Get currents in mA and round to nearest integer
  I1 = round(ina3221.getCurrent_mA(BOARD0, CHAN1));
  I2 = round(ina3221.getCurrent_mA(BOARD0, CHAN2));
  I3 = round(ina3221.getCurrent_mA(BOARD0, CHAN3));
  I4 = round(ina3221.getCurrent_mA(BOARD1, CHAN1));
  I5 = round(ina3221.getCurrent_mA(BOARD1, CHAN2));
  I6 = round(ina3221.getCurrent_mA(BOARD1, CHAN3));
  I7 = round(ina3221.getCurrent_mA(BOARD2, CHAN1));
  I8 = round(ina3221.getCurrent_mA(BOARD2, CHAN2));
  I9 = round(ina3221.getCurrent_mA(BOARD2, CHAN3));

  // Get temperatures - 273.15 for C, × 9/5 - 459.67 for F
  T1 = (float)analogRead(ADC0) * ktemp * 9 / 5 - 459.67;
  T2 = (float)analogRead(ADC1) * ktemp * 9 / 5 - 459.67;
  T3 = (float)analogRead(ADC2) * ktemp * 9 / 5 - 459.67;

  // Reset Zener line ready for next time
  myDigitalWrite(D6, LOW);

// Prepare string for publishing.  Vars temp and voltage to 1 decimal place, current as an integer
/*
  outstring = "";
  outstring = outstring + String(T1, 1) + ", ";
  outstring = outstring + String(T2, 1) + ", ";
  outstring = outstring + String(T3, 1) + ", ";
  outstring = outstring + String(V1, 1) + ", ";
  outstring = outstring + String(I1, 0) + ", ";
  outstring = outstring + String(V2, 1) + ", ";
  outstring = outstring + String(I2, 0) + ", ";
  outstring = outstring + String(V3, 1) + ", ";
  outstring = outstring + String(I3, 0) + ", ";
  outstring = outstring + String(V4, 1) + ", ";
  outstring = outstring + String(I4, 0) + ", ";
  outstring = outstring + String(V5, 1) + ", ";
  outstring = outstring + String(I5, 0) + ", ";
  outstring = outstring + String(V6, 1) + ", ";
  outstring = outstring + String(I6, 0) + ", ";
  outstring = outstring + String(V7, 1) + ", ";
  outstring = outstring + String(I7, 0) + ", ";
  outstring = outstring + String(V8, 1) + ", ";
  outstring = outstring + String(I8, 0) + ", ";
  outstring = outstring + String(V9, 1) + ", ";
  outstring = outstring + String(I9, 0) ;
*/

/* Test values

    I1 = 1234;
    V1 = 18.7;
    I2 = 1234.9;
    V2 = 18.7;
    I3 = 1234;
    V3 = 18.7;
    I4 = 1234;
    V4 = 18.7;
    I5 = 1234;
    V5 = 18.7;
    I6 = 1234;
    V6 = 18.7;
    I7 = 1234;
    V7 = 18.7;
    I8 = 1234;
    V8 = 18.7;
    I9 = 1234;
    V9 = 18.7;
 */

// Subject to Particle limit of 255 bytes / chars
// Tested Max per value is 5 digits: 3 digits, point, one decimal (example 123.4)
// Settings:
// Temps (in F)and Volts are set to one decimal place, which in practice, whether we use C or F, is 4 digits: 78.4
// Current (in milliamps) is set to zero decimal points which in practice is also 4 digits: 1597 Interesting that we save a char by mA, in amps is 5 digits 1.597

//    "\",\"c\":\"" + String("3") +  ->> c is NOT SUPPORTED BY GOOGLE SCRIPTS ARRGGGGG

//COMMENTED SINCE loop is executed every second. I suggest this publish to get its own function and timer ;)
/*
   Particle.publish("pg",
     "{\"a\":\"" + String(T1, 1) +
     "\",\"b\":\"" + String(T2, 1) +
     "\",\"d\":\"" + String(T3, 1) +
     "\",\"e\":\"" + String(V1, 1) +
     "\",\"f\":\"" + String(I1, 0) +
     "\",\"g\":\"" + String(V2, 1) +
     "\",\"h\":\"" + String(I2, 0) +
     "\",\"i\":\"" + String(V3, 1) +
     "\",\"j\":\"" + String(I3, 0) +
     "\",\"k\":\"" + String(V4, 1) +
     "\",\"l\":\"" + String(I4, 0) +
     "\",\"m\":\"" + String(V5, 1) +
     "\",\"n\":\"" + String(I5, 0) +
     "\",\"o\":\"" + String(V6, 1) +
     "\",\"p\":\"" + String(I6, 0) +
     "\",\"q\":\"" + String(V7, 1) +
     "\",\"r\":\"" + String(I7, 0) +
     "\",\"s\":\"" + String(V8, 1) +
     "\",\"t\":\"" + String(I8, 0) +
     "\",\"u\":\"" + String(V9, 1) +
     "\",\"v\":\"" + String(I9, 0) +
     "\"}",
     60, PRIVATE);
 delay(120000);
*/

#ifdef DEBUG_SERIAL
  // If DEBUG is defined, output everything to the serial port
  Serial.println(outstring);
  Serial.print("V1 = ");
  Serial.print(V1);
  Serial.print(" I1 = ");
  Serial.println(I1);
  Serial.print("V2 = ");
  Serial.print(V2);
  Serial.print(" I2 = ");
  Serial.println(I2);
  Serial.print("V3 = ");
  Serial.print(V3);
  Serial.print(" I3 = ");
  Serial.println(I3);
  Serial.print("V4 = ");
  Serial.print(V4);
  Serial.print(" I4 = ");
  Serial.println(I4);
  Serial.print("V5 = ");
  Serial.print(V5);
  Serial.print(" I5 = ");
  Serial.println(I5);
  Serial.print("V6 = ");
  Serial.print(V6);
  Serial.print(" I6 = ");
  Serial.println(I6);
  Serial.print("V7 = ");
  Serial.print(V7);
  Serial.print(" I7 = ");
  Serial.println(I7);
  Serial.print("V8 = ");
  Serial.print(V8);
  Serial.print(" I8 = ");
  Serial.println(I8);
  Serial.print("V9 = ");
  Serial.print(V9);
  Serial.print(" I9 = ");
  Serial.println(I9);
  Serial.print(" T1 = ");
  Serial.println(T1);
  Serial.print(" T2 = ");
  Serial.println(T2);
  Serial.print(" T3 = ");
  Serial.println(T3);
#endif

// Delay before next sample
#ifdef SAMPLETIME
  for (i = 0; i < 1; i++)
    delay(SAMPLETIME);
#endif

  // Test Vals
  /*
T1 = 1.11;
T2 = 2.22;
T3 = 3.33;
V4 = 5.1;
I4 = 1.9;
*/

  mqttLoop();
  // Build the json payload:
  // { "data" : { "tempF" : val, "tempC" : val }}
  StaticJsonBuffer<500> jsonBuffer;

  JsonArray &root = jsonBuffer.createArray();
  JsonObject &data = root.createNestedObject().createNestedObject("data");
  data["T1"] = T1;
  data["T2"] = T2;
  data["T3"] = T3;
  data["V2"] = V2;
  data["I2"] = I2;
  data["V3"] = V3;
  data["I3"] = I3;
  data["V4"] = V4;
  data["I4"] = I4;

  // Get JSON string.
  char buffer[500];
  root.printTo(buffer, sizeof(buffer));
  mqttClient.publish(MQTT_TOPIC_STATE, buffer);

  mqttReconnect();

  delay(1000);
}

int switchToggle(String command)
{
  if (command == "S1ON")
  {
    myDigitalWrite(D2, HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S1 set on");
#endif
    return 1;
  }
  else if (command == "S2ON")
  {
    myDigitalWrite(D3, HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S2 set on");
#endif
    return 2;
  }
  else if (command == "S3ON")
  {
    myDigitalWrite(D4, HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S3 set on");
#endif
    return 3;
  }
  else if (command == "S4ON")
  {
    myDigitalWrite(D5, HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S4 set on");
#endif
    return 4;
  }
  else if (command == "SOFF")
  {
    myDigitalWrite(D2, LOW);
    myDigitalWrite(D3, LOW);
    myDigitalWrite(D4, LOW);
    myDigitalWrite(D5, LOW);
#ifdef DEBUG_SERIAL
    Serial.println("All switches off");
#endif
    return 5;
  }
  else
  {
    return -1;
  }
}

/*******************************************************************************
 * Function Name  : mqttReconnect
 * Description    : reconnects to the mqtt broker if disconnected
 * Return         : true if success, false otherwise
 *******************************************************************************/
void mqttReconnect()
{

  // is time up? no, then come back later
  if (mqttReconnectTimer < MQTT_RECONNECT_INTERVAL)
  {
    return;
  }

  // time is up, reset timer
  mqttReconnectTimer = 0;

   Particle.publish(APP_NAME, "Checking mqtt connection");

  // now try to reconnect
  if (! mqttClient.isConnected())
    mqttConnect();

}

/*******************************************************************************
 * Function Name  : mqttConnect
 * Description    : connects to the mqtt broker and subscribes to interesting topics
 * Return         : true if success, false otherwise
 *******************************************************************************/
bool mqttConnect()
{
  if (mqttClient.isConnected())
    return true;

  mqttClient.connect(LOSANT_DEVICE_ID, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);

  if (mqttClient.isConnected())
  {
    Particle.publish("MQTT connected", "Hurray!", PRIVATE);
  }
  else
  {
    Particle.publish("MQTT failed to connect", "", PRIVATE);
    return false;
  }

  // we need to subscribe to topics that this device is interested in
  mqttSubscribeToInterestingTopics();

  return true;
}

/*******************************************************************************
 * Function Name  : mqttSubscribeToInterestingTopics
 * Description    : subscribes to interesting topics
 * Return         : none
 *******************************************************************************/
void mqttSubscribeToInterestingTopics()
{
  mqttClient.subscribe(MQTT_TOPIC_COMMAND);
}

/*******************************************************************************
 * Function Name  : mqttLoop
 * Description    : processes the mqtt events
 * Return         : true if success, false otherwise
 *******************************************************************************/
bool mqttLoop()
{
  if (mqttClient.isConnected())
  {
    mqttClient.loop();
    return true;
  }

  return false;
}

/*******************************************************************************
 * Function Name  : mqttPublish
 * Description    : publishes the payload to the topic
 * Parameters     : String topic
                    String payload
 * Return         : true if success, false otherwise
 *******************************************************************************/
bool mqttPublish(String topic, String payload)
{
  if (mqttClient.isConnected())
  {
    mqttClient.publish(topic, payload);
    return true;
  }

  return false;
}

/*******************************************************************************
 * Function Name  : mqttCallback
 * Description    : receives publishes sent by the mqtt broker
 * Parameters     : char* topic
                    byte* payload
                    unsigned int length
 * Return         : none
 *******************************************************************************/
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String payloadStr(p);
  String topicStr(topic);

  // Parse the command payload.
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject &command = jsonBuffer.parseObject((char *)payload);

  if (String(command["name"].asString()).equals(String("button1")))
  {
    bool toggler = command["payload"];

    if (toggler)
    {
      myDigitalWrite(D2, HIGH);
    }
    else
    {
      myDigitalWrite(D2, LOW);
    }
    Particle.publish("DEBUG fet1 set to: ", String(toggler), PRIVATE);
  }

  if (String(command["name"].asString()).equals(String("button2")))
  {
    bool toggler = command["payload"];

    if (toggler)
    {
      myDigitalWrite(D3, HIGH);
    }
    else
    {
      myDigitalWrite(D3, LOW);
    }
    Particle.publish("DEBUG fet2 set to: ", String(toggler), PRIVATE);
  }

  if (String(command["name"].asString()).equals(String("button3")))
  {
    bool toggler = command["payload"];

    if (toggler)
    {
      myDigitalWrite(D4, HIGH);
    }
    else
    {
      myDigitalWrite(D4, LOW);
    }
    Particle.publish("DEBUG fet3 set to: ", String(toggler), PRIVATE);
  }

  if (String(command["name"].asString()).equals(String("button4")))
  {
    bool toggler = command["payload"];

    if (toggler)
    {
      myDigitalWrite(D5, HIGH);
    }
    else
    {
      myDigitalWrite(D5, LOW);
    }
    Particle.publish("DEBUG fet4 set to: ", String(toggler), PRIVATE);
  }

  Particle.publish("DEBUG topic/payload", payloadStr + "--" + topicStr, PRIVATE);
}

/*******************************************************************************/
/*******************************************************************************/
/*******************          EEPROM FUNCTIONS         *************************/
/********  https://docs.particle.io/reference/firmware/photon/#eeprom         **/
/********                                                                     **/
/********  wear and tear discussion:                                          **/
/********  https://community.particle.io/t/eeprom-flash-wear-and-tear/23738/5 **/
/**                                                                           **/
/** we can write 200 million bytes to eeprom before it wears out (read link)  **/
/*******************************************************************************/
/*******************************************************************************/

/*******************************************************************************
 * Function Name  : readFromEeprom
 * Description    : retrieves the settings from the EEPROM memory
 * Return         : none
 *******************************************************************************/
void readFromEeprom()
{

  EepromMemoryStructure myObj;
  EEPROM.get(EEPROM_ADDRESS, myObj);

  //verify this eeprom was written before
  // if version is 255 it means the eeprom was never written in the first place, hence the
  // data just read with the previous EEPROM.get() is invalid and we will ignore it
  if (myObj.version == EEPROM_VERSION)
  {

    outputD2 = myObj.outputD2;
    outputD3 = myObj.outputD3;
    outputD4 = myObj.outputD4;
    outputD5 = myObj.outputD5;
    outputD6 = myObj.outputD6;

    Particle.publish(APP_NAME, "Read settings from EEPROM: " + String(outputD2));
  }
  else
  {
    outputD2 = LOW;
    outputD3 = LOW;
    outputD4 = LOW;
    outputD5 = LOW;
    outputD6 = LOW;
  }

}

/*******************************************************************************
 * Function Name  : saveSettingsInEeprom
 * Description    : in this function we wait a bit to give the user time
                    to adjust the right value for them and in this way we try not
                    to save in EEPROM at every little change.
                    Remember that each eeprom writing cycle is a precious and finite resource
 * Return         : none
 *******************************************************************************/
void saveSettingsInEeprom()
{

  //store output state in the struct type that will be saved in the eeprom
  eepromMemory.version = EEPROM_VERSION;
  eepromMemory.outputD2 = outputD2;
  eepromMemory.outputD3 = outputD3;
  eepromMemory.outputD4 = outputD4;
  eepromMemory.outputD5 = outputD5;
  eepromMemory.outputD6 = outputD6;

  //then save
  EEPROM.put(EEPROM_ADDRESS, eepromMemory);

  // Particle.publish(APP_NAME, "Stored settings on EEPROM");
}

/*******************************************************************************
 * Function Name  : myDigitalWrite
 * Description    : writes to the pin and stores in EEPROM
 * Return         : void
 *******************************************************************************/
void myDigitalWrite(int output, int status)
{

  digitalWrite(output, status);

  if (output == D2){
    outputD2 = status;
Particle.publish("DEBUG - myDigitalWrite", "setting d2 to " + String(outputD2));
  }
  if (output == D3){
    outputD3 = status;
  }
  if (output == D4){
    outputD4 = status;
  }
  if (output == D5){
    outputD5 = status;
  }
  if (output == D6){
    outputD6 = status;
  }

  saveSettingsInEeprom();

}

/*******************************************************************************
 * Function Name  : updateOutputs
 * Description    : writes all status to the pins, this gets called right after reading from eeprom
                     at boot time
 * Return         : void
 *******************************************************************************/
void updateOutputs()
{

  digitalWrite(D2, outputD2);
Particle.publish("DEBUG - updateOutputs", "setting d2 to " + String(outputD2));
  digitalWrite(D3, outputD3);
  digitalWrite(D4, outputD4);
  digitalWrite(D5, outputD5);
  digitalWrite(D6, outputD6);

}


int toggleD2(String args)
{

if (outputD2==HIGH){
  myDigitalWrite(D2, LOW);
  Particle.publish("DEBUG - toggleD2", "setting d2 to LOW " + String(outputD2));
}
else
{
  myDigitalWrite(D2, HIGH);
  Particle.publish("DEBUG - toggleD2", "setting d2 to HIGH " + String(outputD2));
}

  return 0;
}
