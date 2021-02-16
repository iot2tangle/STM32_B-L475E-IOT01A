/*
 * enable.h
 *
 *  Created on: 18 gen 2021
 *      Author: UTPM9
 */

#ifndef INC_ENABLE_H_
#define INC_ENABLE_H_
#include "main.h"


#define BLE_OK 0

#define EVENT_STARTUP_SIZE 6
#define ACI_GATT_INIT_COMPLETE_SIZE 7
#define SET_ATTRIBUTES(n) (n)
#define SET_CONTENT_LENGTH(n) (n)





//function for starting the BLE protocol
void ble_init(void);

//function that gets a pending event and save the data in the pointer *container
int fetchBleEvent(uint8_t *container, int size);

//check if the event that was fetched is what I expected
int checkEventResp(uint8_t *event, uint8_t *reference, int size);

void sendCommand(uint8_t *command,int size);

void catchBLE();

void setConnectable();

//set data +- 9999 as maximum
void updateSignedMillesimal(uint8_t *service, uint8_t*characteristic,uint8_t *defaultValue,uint8_t offset, int16_t data);

int BLE_command(uint8_t* command, int size, uint8_t* result, int sizeRes, int returnHandles);

void addService(uint8_t* UUID, uint8_t* handle, int attributes);

void addCharacteristic(uint8_t* UUID,uint8_t* handleChar, uint8_t* handleService, uint8_t maxsize, uint8_t proprieties);

void updateCharValue(uint8_t* handleService,uint8_t* handleChar, int offset, int size,uint8_t* data);

//-+999.9 as value
void updateSignedFloat(uint8_t *service, uint8_t*characteristic,uint8_t *defaultValue,uint8_t offset, float data);
#endif /* INC_ENABLE_H_ */
