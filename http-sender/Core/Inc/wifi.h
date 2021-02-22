/*
 * wifi.h
 *
 *  Created on: 19 feb 2021
 *      Author: Raul Rosa
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include "stdint.h"



#define SSIDcommand "C1=" SSID "\r\n"
#define PSWcommand "C2=" PSW "\r\n"                //command to set the password
#define SECcommad "C3=4\r\n"                      //security of the network WPA2 BEWARE, IF YOUR NETWORK IS NOT WPA2 THIS COMMAND SHOULD BE CHANGED
#define CONNECTcommand "C0\r\n"


#define TCPcommand "P1=0\r\n" //                        TCP
#define IPcommand "P3=" IPgw "\r\n\n"           //  Server IP address (D0 will perform a DNS lookup on and update P3 automatically)
#define PORTcommand "P4=" PORTgw "\r\n\n"           //  Server port
#define STARTTCPcommand "P6=1\r\n"                      //  Start the client
#define CLOSETCPcommand "P6=0\r\n"                       //Stop client


#define STIMOEOUTcommand "S2=1000\r\n\n"
#define TCPSENDcommand "S3="
#define READBUFFERcomand "R1=100\r\n"
#define READcommand "R0\r\n"




#define POSTinit "\rPOST /sensor_data HTTP/1.1\nHost: " IPgw ":" PORTgw "\nContent-Type: application/json\nContent-Length:"
#define POSTmid    "\n\n{ \"iot2tangle\": [ { \"sensor\":"
#define POSTclose  "} ] }], \"device\": \"" deviceid "\", \"timestamp\": 0  }\r\n\r\n"





int checkWifiEvent(uint8_t * input, uint8_t *event, int size);
int getWifiModuleReady();

void startWifi();

void sendCommand(uint8_t *command,int size);
void bigToLittle(uint8_t *input, uint8_t *output, int size);
void littleToBig(uint8_t *input, uint8_t *output, int size);


void setDataInt(uint8_t *payload, int data);
void sendData();
void setDataFloat(uint8_t *payload, float data);
void setSensor(uint8_t *name);


#endif /* INC_WIFI_H_ */
