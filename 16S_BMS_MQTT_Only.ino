/*
* Description :   Software for a 16S BMS based on the BQ76952 and an ESP32. 
* Author      :   James Fotherby
* Date        :   08/04/2025
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*
* Purpose of software:
*     # Detect cell under and over voltage and signal to the Multiplus using 2-signal-BMS
*     # Balance the cells - if we have WiFi and MQTT then balance cells if given permission via NodeRed
*     # Send cell voltages to NodeRed
*
* IT IS VERY IMPORTANT NODE-RED IS CONFIGURED TO STOP ALL MPPTs WHEN THE 2-SIGNAL-BMS REQUESTS - This BMS has no direct control of the MPPTs
*
* In normal operation the BMS should never have to take action. The inverter or MPPT charger should have more conservative limits to shut off first. For 
* example the MPPT charger is set to charge to 53.6 volts (90% SOC) which is a cell voltage of 3.35 volts. This is far from the BMS cell overvoltage threshold 
* of 3.5 volts. Likewise the inverter is set to discharge to 48.0 volts (10% SOC) which is 3.00 volts per cell and far from the BMS under voltage cut out of
* 2.75 volts.
*
*
* To Do:
*   - Send individual cell voltages - make a copy, then send 4/messages a second until 16 sent. do this once every minute
*   - influxdb from node red
* 
*/

#include <BQ76952.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  // v6+
#include "OpenBMS_James.h"

//------------- Pin definitions --------------------
#define LED_GREEN_PIN       0
#define LED_RED_PIN         1

#define CAN_RX              2
#define CAN_TX              3

#define BUTTON_LOW_PIN      9
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9                                       // Pulling to ground and reseting device enters BOOT mode
#define BOOT_BTN_PIN        I2C_SCL_PIN

#define AUX_1_PIN           10                                      // Use Aux 1 to disable the Multiplus II GX inverter
#define AUX_3_PIN           11                                      // Use Aux 3 to disable the SmartSolar MPPT charger
#define AUX_2_PIN           12

#define UART_TX             16
#define UART_RX             17

#define VE_RX               19
#define VE_TX               20

//------------- Other definitions --------------------
#define NUM_OF_CELLS        16

#define CUV_Delay				    0x9276                                  // [Default: 74 * 3.3 ms = 250 ms]
#define COV_Delay				    0x9279                                  // [Default: 74 * 3.3 ms = 250 ms]

// Your credentials
const char* ssid = "venus-HQ2306ZHQGJ-f3b";         
const char* password = "zz6tzefc";                 
const char* mqtt_server = "192.168.188.122";                 

// Time intervals in milliseconds
const unsigned long TaskAInterval = 200;        // Runs client.loop(); and blinks balancing LED when balancing cells
const unsigned long TaskBInterval = 5000;       // maintainConnections(); Reads battery metrics. Prints to debug, drives optoisolators
const unsigned long TaskCInterval = 15000;      // allows/disallows balancing based on MQTT control or timeout 
const unsigned long TaskDInterval = 60000;      // Sends cell voltage JSON over MQTT
const unsigned long TaskEInterval = 900000;     // Sends cell balancing cumulative times JSON over MQTT

const unsigned long MQTT_Timeout  = 30000;      // Time without recieving MQTT from node-red before we allow balancing 

BQ76952             bms;
WiFiClient          espClient;
PubSubClient        client(espClient);

// Flags to track first-time config
bool              mqttConfigured = false;
bool              allowbalancing = true;
uint32_t          Last_MQTT_Balancing_Packet = 0;

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
void setup() {
  Serial.begin(115200);
  bms.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  bms.reset();
  delay(100); 

  // Configure the BQ76952:
  bms.setConnectedCells(NUM_OF_CELLS);  
  bms.writeByteToMemory(CUV_Threshold, 55);                         // Set Cell undervoltage protection to 2.75 volts (units: 50.6 mV)
  bms.writeByteToMemory(COV_Threshold, 70);                         // Set Cell overvoltage protection to 3.50 volts 
  bms.writeIntToMemory(CUV_Delay, 1513);                            // 5 Second delay before a CUV triggers a fault
  bms.writeIntToMemory(COV_Delay, 1513);

  bms.writeByteToMemory(Enabled_Protections_A, 0b00001100);         // Enable CUV, COV circuit protection
  bms.writeByteToMemory(Enabled_Protections_B, 0b01000000);         // Enable internal overtemp protection 80-65 celcius

  bms.writeIntToMemory(Cell_Balance_Min_Cell_V_Relaxed, 3350);      // Only balance when cells above 3350 mV (Top balance)
  bms.writeByteToMemory(Cell_Balance_Max_Cells, 8);                 // Allows up to 8 simultaneous cells to be balanced
  bms.writeByteToMemory(Balancing_Configuration, 0b00000010);       // Enables autonomous balancing in realxed state (CB_RLX)
  bms.writeIntToMemory(Power_Config, 0b0010100010110010);           // Disables SLEEP, slow measurement speed when balancing to 1/8th

  bms.writeByteToMemory(FET_Options, 0b00000000);                   // FETs will not be turned on
  bms.writeByteToMemory(Chg_Pump_Control, 0b00000000);              // Disable charge pump as not needed
  bms.writeIntToMemory(Mfg_Status_Init, 0x0000);                    // The BQ doesn't control any FETs.

  // Configure pins
  pinMode(LED_GREEN_PIN, OUTPUT); digitalWrite(LED_GREEN_PIN, HIGH); 
  pinMode(LED_RED_PIN, OUTPUT); digitalWrite(LED_RED_PIN, HIGH);

  pinMode(AUX_1_PIN, OUTPUT); digitalWrite(AUX_1_PIN, LOW);
  pinMode(AUX_2_PIN, OUTPUT); digitalWrite(AUX_2_PIN, LOW);
  pinMode(AUX_3_PIN, OUTPUT); digitalWrite(AUX_3_PIN, LOW);

  maintainConnections();
}

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
// Callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);

  Serial.print(", Payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, "Balancing/permission") == 0) {
    Last_MQTT_Balancing_Packet = millis();

    if ((char)payload[0] == 'N' && (char)payload[1] == 'o') {
      allowbalancing = false;
    } else {
      allowbalancing = true;
    }
  }
}

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
void maintainConnections() {
  // Attempt WiFi connection if not connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Attempting reconnect...");
    WiFi.begin(ssid, password);                                     // Will return immediately
    return;                                                         // Don't continue to MQTT until WiFi is back
  }

  // Configure MQTT once when WiFi is up
  if (!mqttConfigured) {
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    mqttConfigured = true;
  }

  // Attempt MQTT reconnect if not connected
  if (!client.connected()) {
    Serial.println("MQTT not connected. Attempting reconnect...");
    if (client.connect("ESP32Client")) {
      Serial.println("MQTT connected");
      client.subscribe("Balancing/permission");                     // Subscribe to topics here
    } else {
      Serial.print("MQTT connect failed. State=");
      Serial.println(client.state());                               // Don't block; try again next time
    }
  }  
}

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
void loop() { 
  static uint16_t Cell_Voltages[17];
  static uint32_t Cell_Balance_Times[17]; 
  unsigned long currentMillis = millis();

  delay(50);

  //----------------------------------------------------------------------------------------------------------------------  
  static unsigned long TaskA_timestamp = 0;
  if (currentMillis - TaskA_timestamp >= TaskAInterval) {
    TaskA_timestamp = currentMillis; 
    
    client.loop();

    if(bms.isBalancing()) {
      digitalWrite(LED_GREEN_PIN, !digitalRead(LED_GREEN_PIN));   
    }
  }

  //---------------------------------Read Cell Metrics------------------------------------------------------------------  
  static unsigned long TaskB_timestamp = 0;
  if (currentMillis - TaskB_timestamp >= TaskBInterval) {
    TaskB_timestamp = currentMillis; 

    maintainConnections();
    digitalWrite(LED_GREEN_PIN, !digitalRead(LED_GREEN_PIN));       // Heartbeat LED 

    int8_t Alert_A = bms.directCommandRead(DIR_CMD_FPROTEC);        // Read the protection registers
    int8_t Alert_B = bms.directCommandRead(DIR_CMD_FTEMP);
	
    if(Alert_A & 0b0100)	{					                                // CUV Fault - disable inverter
      digitalWrite(AUX_1_PIN, LOW);		
    }
    else {
      digitalWrite(AUX_1_PIN, HIGH);	
    }
      
    if(Alert_A & 0b1000)	{					                                // COV Fault - disable charging
      digitalWrite(AUX_2_PIN, LOW);
    }
    else {
      digitalWrite(AUX_2_PIN, HIGH);	
    }
    
    // Drive our fault LED
    if(Alert_A)	{                                                   // If there is a fault light up the red LED! On = COV, Slow blink = CUV
      if(Alert_A & 0b1000)	{
        digitalWrite(LED_RED_PIN, HIGH);
      }
      else if(Alert_A & 0b0100) {
        digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN)); 
      }
    }
    else {
      digitalWrite(LED_RED_PIN, LOW);
    }	

    // Print out Faults:
    Serial.print("Fault A: "); Serial.print(Alert_A, BIN); 
    Serial.print(", Fault B: "); Serial.println(Alert_B, BIN);

    // Read and Print out all individual cell voltages
    Serial.print("Cell Voltages: ");
    for(byte i = 0;i < NUM_OF_CELLS;i++)  {
      Cell_Voltages[i] = bms.getCellVoltage(i+1);
      Serial.print(Cell_Voltages[i]); Serial.print(", ");
    }
    Serial.println();

    // Print out all the cell balance times
    Serial.print("Cell Balance Times: ");
    bms.GetCellBalancingTimes(Cell_Balance_Times);
    for (int i = 0; i < NUM_OF_CELLS; i++) {
      Cell_Balance_Times[i] = Cell_Balance_Times[i + 1];  // shift left by 1
      Serial.print(Cell_Balance_Times[i]); Serial.print(", ");
    }
    Serial.println();

    // Print out temperatures 
    Serial.print("Temperatures: ");   
    Serial.print(bms.getThermistorTemp(TS1)); Serial.print(", ");  
    Serial.println(bms.getInternalTemp());
    Serial.println();
  }

  //---------------------------------------------------------------------------------------------------------------------- 
  static unsigned long TaskC_timestamp = 0;
  if (currentMillis - TaskC_timestamp >= TaskCInterval) {
    TaskC_timestamp = currentMillis;

    // If no MQTT messages arriving assume control from NodeRed is lost and enable balancing
    if (currentMillis - Last_MQTT_Balancing_Packet >= MQTT_Timeout) {
      allowbalancing = true;
    }

    if(allowbalancing == true)  {
      bms.writeByteToMemory(Balancing_Configuration, 0b00000010);       // Enables autonomous balancing
      Serial.println("Cell balancing Enabled");
    }
    else  {
      bms.writeByteToMemory(Balancing_Configuration, 0b00000000);       // Disables autonomous balancing
      Serial.println("Cell balancing Disabled");
    }
  }

  //---------------------------------------------------------------------------------------------------------------------- 
  static unsigned long TaskD_timestamp = 0;
  if (currentMillis - TaskD_timestamp >= TaskDInterval) {
    TaskD_timestamp = currentMillis;

    char voltageJson[256];
    StaticJsonDocument<300> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < NUM_OF_CELLS; i++) {
      arr.add(Cell_Voltages[i]);
    }
    serializeJson(doc, voltageJson);
    Serial.println(voltageJson);
    client.publish("battery/cell_voltages", voltageJson);  
  }

  //---------------------------------------------------------------------------------------------------------------------- 
  static unsigned long TaskE_timestamp = 0;
  if (currentMillis - TaskE_timestamp >= TaskEInterval) {
    TaskE_timestamp = currentMillis;

    char balanceJson[512];
    StaticJsonDocument<600> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < NUM_OF_CELLS; i++) {
      arr.add(Cell_Balance_Times[i]);
    }
    serializeJson(doc, balanceJson);
    Serial.println(balanceJson);
    client.publish("battery/balance_times", balanceJson); 
  }  
}













