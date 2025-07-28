/*
* Description :   Software for a 16S BMS based on the BQ76952 and an ESP32. 
* Author      :   James Fotherby
* Date        :   08/04/2025
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#ifndef OpenBMS_James_h
#define OpenBMS_James_h

#include "Arduino.h"

#define GREEN                 0
#define RED                   1

// Function prototypes:
void maintainConnections(void);
void callback(char* topic, byte* payload, unsigned int length);
void publishCellData();

#endif
