/*
* Description :   Software for a 16S BMS based on the BQ76952 and an ESP32. 
* Author      :   James Fotherby
* Date        :   31/07/2025
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*
* Purpose of software:
*     # Detect cell under and over voltage and signal to the Multiplus using the 2-signal-BMS assistant 
*     # Balance the cells - if we have WiFi and MQTT then balance cells if given permission via NodeRed
*     # Send cell voltages to NodeRed
*
* IT IS VERY IMPORTANT NODE-RED IS CONFIGURED TO STOP ALL MPPTs WHEN THE 2-SIGNAL-BMS REQUESTS - This BMS has no direct control of the MPPTs
*
* In normal operation the BMS should never have to take action. The inverter or MPPT charger(s) should have more conservative limits to shut off first. For 
* example the MPPT charger is set to charge to 53.6 volts (90% SOC) which is a cell voltage of 3.35 volts. This is far from the BMS cell overvoltage threshold 
* of 3.5 volts. Likewise the inverter is set to discharge to 48.0 volts (10% SOC) which is 3.00 volts per cell and far from the BMS under voltage cut out of
* 2.75 volts.
*
* To do:
*  - We made some changes to the library and added a new function. We need to test whether this new function works.
*/

#include <BQ76952.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>                                            // v6+
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
#define AUX_3_PIN           11                                      
#define AUX_2_PIN           12                                      // Use Aux 2 to disable the SmartSolar MPPT charger

#define UART_TX             16
#define UART_RX             17

#define VE_RX               19
#define VE_TX               20

//------------- Other definitions --------------------
#define NUM_OF_CELLS        16
#define Min_Cell_Temp       0x9336                                  

// Your credentials
const char* ssid = "venus-HQ2306ZHQGJ-f3b";         
const char* password = "zz6tzefc";                 
const char* mqtt_server = "192.168.188.122";                

// Time intervals in milliseconds
const unsigned long TaskAInterval = 200;        // Runs client.loop(); and blinks balancing LED when balancing cells
const unsigned long TaskBInterval = 5000;       // maintainConnections(); Reads battery metrics. Prints to debug, drives optoisolators     
const unsigned long TaskDInterval = 60000;      // Sends cell voltage JSON over MQTT
const unsigned long TaskEInterval = 900000;     // Sends cell balancing cumulative times JSON over MQTT

BQ76952             bms;
WiFiClient          espClient;
PubSubClient        client(espClient);

// Flags to track first-time config
bool              mqttConfigured = false;

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
void setup() {
  delay(1000);
  Serial.begin(115200);
  bms.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  bms.reset();
  delay(100); 

  // Configure the BQ76952:
  bms.setConnectedCells(NUM_OF_CELLS);  

  bms.writeByteToMemory(CUV_Threshold, 55);                         // Set Cell undervoltage protection to 2.75 volts (units: 50.6 mV)
  bms.writeByteToMemory(COV_Threshold, 68);                         // Set Cell overvoltage protection to 3.50 volts 
  bms.writeIntToMemory(CUV_Delay, 1513);                            // 5 Second delay before a CUV/COV triggers a fault. Default 100 mV hysterisis
  bms.writeIntToMemory(COV_Delay, 1513);

  bms.writeByteToMemory(Enabled_Protections_A, 0b00001100);         // Enable CUV, COV circuit protection
  bms.writeByteToMemory(Enabled_Protections_B, 0b01000000);         // Enable internal overtemp protection 80-65 celcius

  bms.writeIntToMemory(Cell_Balance_Min_Cell_V_Relaxed, 3350);      // Only balance when cells above 3350 mV (Top balance)
  bms.writeByteToMemory(Cell_Balance_Max_Cells, 8);                 // Allows up to 8 simultaneous cells to be balanced
  bms.writeByteToMemory(DA_Configuration, 0b00001101);              // Set chip die to be for cell temp measurment

  //bms.writeByteToMemory(Min_Cell_Temp, -127);
  bms.writeByteToMemory(TS1_Config, 0b00000000);                    // Configures TS1 not to be used (By default it's used as a cell temp input)
  bms.writeByteToMemory(Balancing_Configuration, 0b00000011);       // Enables autonomous balancing
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
    if ((char)payload[0] == 'N' && (char)payload[1] == 'o') {
      uint8_t payload[2] = {0, 0};
      bms.subCommandWriteData(CB_ACTIVE_CELLS, payload, sizeof(payload));         // Disables balancing for Cell Balance Interval (default 20 seconds)   
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
  static uint16_t Cell_Voltages[NUM_OF_CELLS];
  static uint32_t Cell_Balance_Times[NUM_OF_CELLS]; 
  unsigned long currentMillis = millis();

  delay(1);

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
      Serial.print(Cell_Balance_Times[i]); Serial.print(", ");
    }
    Serial.println();

    // Print out temperatures 
    Serial.print("Temp: ");     
    Serial.println(bms.getInternalTemp());
    Serial.println();
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
    serializeJson(doc, voltageJson, sizeof(voltageJson));
    Serial.println(voltageJson);
    client.publish("battery/cell_voltages", voltageJson); 

  //-----------------------------------------------------------------------------------------
    byte* voltage_data = bms.subCommandwithdata(DASTATUS_5, 32);            

    // Define variables for each data item
    uint16_t VREG18 = (voltage_data[1] << 8) | voltage_data[0]; // 16-bit
    uint16_t VSS = (voltage_data[3] << 8) | voltage_data[2];    // 16-bit
    uint16_t MaxCellVoltage = (voltage_data[5] << 8) | voltage_data[4]; // 16-bit
    uint16_t MinCellVoltage = (voltage_data[7] << 8) | voltage_data[6]; // 16-bit
    uint16_t BatteryVoltageSum = (voltage_data[9] << 8) | voltage_data[8]; // 16-bit
    uint16_t AvgCellTemperature = (voltage_data[11] << 8) | voltage_data[10]; // 16-bit
    uint16_t FETTemperature = (voltage_data[13] << 8) | voltage_data[12]; // 16-bit
    uint16_t MaxCellTemperature = (voltage_data[15] << 8) | voltage_data[14]; // 16-bit
    uint16_t MinCellTemperature = (voltage_data[17] << 8) | voltage_data[16]; // 16-bit

    // Print the extracted data for verification
    Serial.println(F("Extracted Voltage Data:"));
    Serial.print(F("VREG18: ")); Serial.println(VREG18);
    Serial.print(F("VSS: ")); Serial.println(VSS);
    Serial.print(F("Max Cell Voltage: ")); Serial.println(MaxCellVoltage);
    Serial.print(F("Min Cell Voltage: ")); Serial.println(MinCellVoltage);
    Serial.print(F("Battery Voltage Sum: ")); Serial.println(BatteryVoltageSum);
    Serial.print(F("Avg Cell Temperature: ")); Serial.println(AvgCellTemperature);
    Serial.print(F("FET Temperature: ")); Serial.println(FETTemperature);
    Serial.print(F("Max Cell Temperature: ")); Serial.println(MaxCellTemperature);
    Serial.print(F("Min Cell Temperature: ")); Serial.println(MinCellTemperature);
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
    serializeJson(doc, balanceJson, sizeof(balanceJson));
    Serial.println(balanceJson);
    client.publish("battery/balance_times", balanceJson); 
  }  
}

