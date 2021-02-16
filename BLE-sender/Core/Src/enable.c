/*
 * enable.c
 *
 *  Created on: 18 gen 2021
 *      Author: UTPM9
 */

#include "enable.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include "ble_commands.h"

extern SPI_HandleTypeDef hspi3;
extern int dataAvailable;
uint8_t deviceName[]={'S','T','M','3','2','T','o','T','a','n','g','l','e'};
//char deviceName[]={'S','T','M','3','2'};//NOT REVERSED, INCREDIBLE STRINGS ARE NOT LITTLE ENDIAN
//attributes are 1 for service, 2 for readable char and 3 for notify/readable characteristcs


uint8_t buffer[255];



uint8_t UUID_SERVICE_1[]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};//Reversed UUID of the service
uint8_t CUSTOM_SERVICE_HANDLE[2];



uint8_t UUID_CHAR_1[]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t CUSTOM_CHAR_HANDLE[2];
uint8_t VALUE1[]={'{','\"','N','a','m','e','\"',':','\"','E','n','v','i','r','o','n','m','e','n','t','\"','}'};



uint8_t UUID_CHAR_2[]={0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t MIC_CHAR_HANDLE[2];
uint8_t VALUE2[]={'{','\"','M','i','c','r','o','p','h','o','n','e','\"',':','\"','+','0','0','0','0','\"','}'};


uint8_t UUID_CHAR_TEMP[]={0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t TEMP_CHAR_HANDLE[2];
uint8_t VALUE_TEMP[]={'{','\"','T','e','m','p','"',':','\"','+','0','0','0','.','0','\"','}'};

//uno meno di temp in lunghezza
uint8_t UUID_CHAR_HUM[]={0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t HUM_CHAR_HANDLE[2];
uint8_t VALUE_HUM[]={'{','\"','H','u','m','"',':','\"','+','0','0','0','.','0','\"','}'};


//uno meno di temp in lunghezza
uint8_t UUID_CHAR_PRESS[]={0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t PRESS_CHAR_HANDLE[2];
uint8_t VALUE_PRESS[]={'{','\"','P','r','e','s','s','"',':','\"','+','0','0','0','.','0','\"','}'};



uint8_t UUID_INERTIAL_SERVICE[]={0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t INERTIAL_SERVICE_HANDLE[2];


uint8_t UUID_CHAR_INERTIAL_NAME[]={0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t NAME_INERTIAL_HANDLE[2];
uint8_t NAME_INERTIAL_VALUE[]={'{','\"','N','a','m','e','\"',':','\"','A','c','c','e','l','e','r','o','m','e','t','e','r','\"','}'};



uint8_t UUID_CHAR_INERTIAL_ACCX[]={0x01,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t ACCX_CHAR_HANDLE[2];
uint8_t ACCX_INERTIAL_VALUE[]={'{','\"','A','x','i','s','X','\"',':','\"','+','0','0','0','0','\"','}'};

uint8_t UUID_CHAR_INERTIAL_ACCY[]={0x02,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t ACCY_CHAR_HANDLE[2];
uint8_t ACCY_INERTIAL_VALUE[]={'{','\"','A','x','i','s','Y','\"',':','\"','+','0','0','0','0','\"','}'};

uint8_t UUID_CHAR_INERTIAL_ACCZ[]={0x03,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t ACCZ_CHAR_HANDLE[2];
uint8_t ACCZ_INERTIAL_VALUE[]={'{','\"','A','x','i','s','Z','\"',':','\"','+','0','0','0','0','\"','}'};

uint8_t X_VALUE[]={'{','\"','A','x','i','s','X','\"',':','\"','+','0','0','0','0','\"','}'};
uint8_t Y_VALUE[]={'{','\"','A','x','i','s','Y','\"',':','\"','+','0','0','0','0','\"','}'};
uint8_t Z_VALUE[]={'{','\"','A','x','i','s','Z','\"',':','\"','+','0','0','0','0','\"','}'};
uint8_t MIC_VALUE[]={'{','\"','M','i','c','r','o','p','h','o','n','e','\"',':','\"','+','0','0','0','0','\"','}'};


uint8_t UUID_MAGNETIC_SERVICE[]={0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t MAGNETIC_SERVICE_HANDLE[2];

uint8_t UUID_CHAR_MAGNETIC_NAME[]={0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t NAME_MAGNETIC_HANDLE[2];
uint8_t NAME_MAGNETIC_VALUE[]={'{','\"','N','a','m','e','\"',':','\"','M','a','g','n','e','t','o','m','e','t','e','r','\"','}'};

uint8_t UUID_CHAR_MAGNETIC_MAGX[]={0x01,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t MAGX_CHAR_HANDLE[2];

uint8_t UUID_CHAR_MAGNETIC_MAGY[]={0x02,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t MAGY_CHAR_HANDLE[2];

uint8_t UUID_CHAR_MAGNETIC_MAGZ[]={0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t MAGZ_CHAR_HANDLE[2];




uint8_t UUID_GYROSCOPE_SERVICE[]={0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t GYROSCOPE_SERVICE_HANDLE[2];

uint8_t UUID_CHAR_GYROSCOPE_NAME[]={0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t NAME_GYROSCOPE_HANDLE[2];
uint8_t NAME_GYROSCOPE_VALUE[]={'{','\"','N','a','m','e','\"',':','\"','G','y','r','o','s','c','o','p','e','\"','}'};

uint8_t UUID_CHAR_GYROSCOPE_GYROX[]={0x01,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t GYROX_CHAR_HANDLE[2];

uint8_t UUID_CHAR_GYROSCOPE_GYROY[]={0x02,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t GYROY_CHAR_HANDLE[2];

uint8_t UUID_CHAR_GYROSCOPE_GYROZ[]={0x03,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t GYROZ_CHAR_HANDLE[2];


uint8_t UUID_TOF_SERVICE[]={0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t TOF_SERVICE_HANDLE[2];

uint8_t UUID_CHAR_TOF_NAME[]={0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t NAME_TOF_HANDLE[2];
uint8_t NAME_TOF_VALUE[]={'{','\"','N','a','m','e','\"',':','\"','T','i','m','e','O','f','F','l','i','g','h','t','\"','}'};

uint8_t UUID_CHAR_TOF_VALUE[]={0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00};
uint8_t TOF_CHAR_HANDLE[2];
uint8_t TOF_VALUE[]={'{','\"','D','i','s','t','a','n','c','e','\"',':','\"','+','0','0','0','0','\"','}'};



uint16_t stackInitCompleteFlag=0;
uint8_t* rxEvent;

void ble_init(){



	//fetching the reset event
	rxEvent=(uint8_t*)malloc(EVENT_STARTUP_SIZE);
	int res;

	while(!dataAvailable);
	res=fetchBleEvent(rxEvent,EVENT_STARTUP_SIZE);

	if(res==BLE_OK){
	res=checkEventResp(rxEvent,EVENT_STATUP_DATA,EVENT_STARTUP_SIZE);
	if(res==BLE_OK){
	   stackInitCompleteFlag|=0x01;
	}
	}
	HAL_Delay(10);
	free(rxEvent);



	//INIT GATT


	if(BLE_command(ACI_GATT_INIT,sizeof(ACI_GATT_INIT),ACI_GATT_INIT_COMPLETE,sizeof(ACI_GATT_INIT_COMPLETE),0)==BLE_OK){
	   stackInitCompleteFlag|=0x02;
	}
	free(rxEvent);


	//INIT GAP, actually the handle that i get is a GATT handle of a service, will change the name later

	if(BLE_command(ACI_GAP_INIT,sizeof(ACI_GAP_INIT),ACI_GAP_INIT_COMPLETE,sizeof(ACI_GAP_INIT_COMPLETE),3)==BLE_OK){
	   stackInitCompleteFlag|=0x04;
	   memcpy(GAP_SERVICE_HANDLE,rxEvent+7,2);
	   memcpy(GAP_CHAR_NAME_HANDLE,rxEvent+9,2);
	   memcpy(GAP_CHAR_APP_HANDLE,rxEvent+11,2);
	}
	free(rxEvent);





	//SET THE NAME OF THE BOARD IN THE SERVICE CREATED AUTOMATICALLY
	updateCharValue(GAP_SERVICE_HANDLE,GAP_CHAR_NAME_HANDLE,0,sizeof(deviceName),deviceName);
	stackInitCompleteFlag|=0x08;
	free(rxEvent);


	//INIT AUTH


	if(BLE_command(ACI_GAP_SET_AUTH,sizeof(ACI_GAP_SET_AUTH),ACI_GAP_SET_AUTH_RESP,sizeof(ACI_GAP_SET_AUTH_RESP),0)==BLE_OK){
	   stackInitCompleteFlag|=0x10;
	}
	free(rxEvent);




	//SET_TX_LEVEL

	if(BLE_command(ACI_HAL_SET_TX_POWER_LEVEL,sizeof(ACI_HAL_SET_TX_POWER_LEVEL),ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE,sizeof(ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE),0)==BLE_OK){
	   stackInitCompleteFlag|=0x20;
	}
	free(rxEvent);



	//SET SCAN RESPONSE DATA

	if(BLE_command(HCI_LE_SET_SCAN_RESPONSE_DATA,sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA),HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE,sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE),0)==BLE_OK){
	   stackInitCompleteFlag|=0x40;
	}
	free(rxEvent);


	//This will start the advertisment,
	setConnectable();


	//let's add the first service
	addService(UUID_SERVICE_1,CUSTOM_SERVICE_HANDLE,SET_ATTRIBUTES(1+2+3*2+3+3));//1 atribute service +2 attribute char readable+3*(2 NOTIFYABLE READABLE charachteristics)

	//add the charachteristic of the name of the services
	addCharacteristic(UUID_CHAR_1,CUSTOM_CHAR_HANDLE,CUSTOM_SERVICE_HANDLE,SET_CONTENT_LENGTH(22),READABLE);


	//set the characteristic
	updateCharValue(CUSTOM_SERVICE_HANDLE,CUSTOM_CHAR_HANDLE,0,SET_CONTENT_LENGTH(22),VALUE1);




	//add the temp charachteristic
	addCharacteristic(UUID_CHAR_TEMP,TEMP_CHAR_HANDLE,CUSTOM_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);

	//set temperature
	updateCharValue(CUSTOM_SERVICE_HANDLE,TEMP_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),VALUE_TEMP);

	//add the hum charachteristic
	addCharacteristic(UUID_CHAR_HUM,HUM_CHAR_HANDLE,CUSTOM_SERVICE_HANDLE,SET_CONTENT_LENGTH(16),READABLE|NOTIFIBLE);

	//set hum
	updateCharValue(CUSTOM_SERVICE_HANDLE,HUM_CHAR_HANDLE,0,SET_CONTENT_LENGTH(16),VALUE_HUM);

	//add the press charachteristic
	addCharacteristic(UUID_CHAR_PRESS,PRESS_CHAR_HANDLE,CUSTOM_SERVICE_HANDLE,SET_CONTENT_LENGTH(18),READABLE|NOTIFIBLE);

	//set press
	updateCharValue(CUSTOM_SERVICE_HANDLE,PRESS_CHAR_HANDLE,0,SET_CONTENT_LENGTH(18),VALUE_PRESS);

	//add the second charachteristic, TOF sensor
	addCharacteristic(UUID_CHAR_TOF_VALUE,TOF_CHAR_HANDLE,CUSTOM_SERVICE_HANDLE,SET_CONTENT_LENGTH(20),READABLE|NOTIFIBLE);

	//setchar2
	updateCharValue(CUSTOM_SERVICE_HANDLE,TOF_CHAR_HANDLE,0,SET_CONTENT_LENGTH(20),TOF_VALUE);







	//let's add the second service
	 addService(UUID_INERTIAL_SERVICE,INERTIAL_SERVICE_HANDLE,SET_ATTRIBUTES(1+2+3+3+3));//1 the service 2 the readable char e 3x3 the readable notifiable chars

	//add the charachteristic of the name
	 addCharacteristic(UUID_CHAR_INERTIAL_NAME,NAME_INERTIAL_HANDLE,INERTIAL_SERVICE_HANDLE,SET_CONTENT_LENGTH(24),READABLE);


	 //set the characteristic
	updateCharValue(INERTIAL_SERVICE_HANDLE,NAME_INERTIAL_HANDLE,0,SET_CONTENT_LENGTH(24),NAME_INERTIAL_VALUE);


	//add the charachteristic of the accelerometer
	 addCharacteristic(UUID_CHAR_INERTIAL_ACCX,ACCX_CHAR_HANDLE,INERTIAL_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);


	 //set the characteristic
	updateCharValue(INERTIAL_SERVICE_HANDLE,ACCX_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),ACCX_INERTIAL_VALUE);


	//add the charachteristic of the accelerometer Y
	 addCharacteristic(UUID_CHAR_INERTIAL_ACCY,ACCY_CHAR_HANDLE,INERTIAL_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);


	 //set the characteristic
	updateCharValue(INERTIAL_SERVICE_HANDLE,ACCY_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),ACCY_INERTIAL_VALUE);

	//add the charachteristic of the accelerometer Z
	 addCharacteristic(UUID_CHAR_INERTIAL_ACCZ,ACCZ_CHAR_HANDLE,INERTIAL_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);


	 //set the characteristic
	updateCharValue(INERTIAL_SERVICE_HANDLE,ACCZ_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),ACCZ_INERTIAL_VALUE);






	//let's add the third service
	 addService(UUID_MAGNETIC_SERVICE,MAGNETIC_SERVICE_HANDLE,SET_ATTRIBUTES(1+2+3*3));//1 the service 2 the readable char e 3x3 the readable notifiable chars

	//add the charachteristic of the name
	 addCharacteristic(UUID_CHAR_MAGNETIC_NAME,NAME_MAGNETIC_HANDLE,MAGNETIC_SERVICE_HANDLE,SET_CONTENT_LENGTH(23),READABLE);


	 //set the characteristic
	updateCharValue(MAGNETIC_SERVICE_HANDLE,NAME_MAGNETIC_HANDLE,0,SET_CONTENT_LENGTH(23),NAME_MAGNETIC_VALUE);


	//add the charachteristic of the accelerometer
	 addCharacteristic(UUID_CHAR_MAGNETIC_MAGX,MAGX_CHAR_HANDLE,MAGNETIC_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);


	 //set the characteristic
	updateCharValue(MAGNETIC_SERVICE_HANDLE,MAGX_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),X_VALUE);

	//add the charachteristic of the accelerometer
	 addCharacteristic(UUID_CHAR_MAGNETIC_MAGY,MAGY_CHAR_HANDLE,MAGNETIC_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);


	 //set the characteristic
	updateCharValue(MAGNETIC_SERVICE_HANDLE,MAGY_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),Y_VALUE);

	//add the charachteristic of the accelerometer
	 addCharacteristic(UUID_CHAR_MAGNETIC_MAGZ,MAGZ_CHAR_HANDLE,MAGNETIC_SERVICE_HANDLE,SET_CONTENT_LENGTH(17),READABLE|NOTIFIBLE);


	 //set the characteristic
	updateCharValue(MAGNETIC_SERVICE_HANDLE,MAGZ_CHAR_HANDLE,0,SET_CONTENT_LENGTH(17),Z_VALUE);











	//TOF SERVICE

//	//add service of gyroscope
//	addService(UUID_TOF_SERVICE,TOF_SERVICE_HANDLE,1+2);
//	//add name service
//	addCharacteristic(UUID_CHAR_TOF_NAME,NAME_TOF_HANDLE,TOF_SERVICE_HANDLE,SET_CONTENT_LENGTH(23),READABLE);
//	//set name
//	updateCharValue(TOF_SERVICE_HANDLE,NAME_TOF_HANDLE,0,SET_CONTENT_LENGTH(23),NAME_TOF_VALUE);






	if(stackInitCompleteFlag==255){
	  //turn on led blue if everything was fine
	//  HAL_GPIO_WritePin(CPU_LED_GPIO_Port,CPU_LED_Pin,GPIO_PIN_SET);
	}
	return;
}

int fetchBleEvent(uint8_t *container, int size){

  uint8_t master_header[]={0x0b,0x00,0x00,0x00,0x00};
  uint8_t slave_header[5];

  //Wait until it is available an event coming from the BLE module (GPIO PIN COULD CHANGE ACCORDING TO THE BOARD)
  if(HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){

  HAL_Delay(5);
  //PIN_CS of SPI2 LOW
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

  //SPI2 in this case, it could change according to the board
  //we send a byte containing a request of reading followed by 4 dummy bytes
  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);

  //let's get the size of data available
  int dataSize;
  dataSize=(slave_header[3]|slave_header[4]<<8);
  int i;
  char dummy=0xff;

  if(dataSize>size){
	  dataSize=size;
  }

  if(dataSize>0){
	    //let's fill the get the bytes availables and insert them into the container variable
  		for(i=0;i<dataSize;i++){
  		HAL_SPI_TransmitReceive(&hspi3,(uint8_t*)&dummy,container+i,1,1);

  		}
  		HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
  	}else{
  		HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
        return -1;
  	}

  //let's stop the SPI2


  dataAvailable=0;
  return BLE_OK;
  }else{
  return -2;
  }
}


int checkEventResp(uint8_t *event, uint8_t *reference, int size){
	int j=0;

	for(j=0;j<size;j++){

		if(event[j]!=reference[j]){
			return -1;
		}
	}

return BLE_OK;
}

//TODO make it not blocking function
void sendCommand(uint8_t *command,int size){

	  uint8_t master_header[]={0x0a,0x00,0x00,0x00,0x00};
	  uint8_t slave_header[5];

	  int result;

	do{


	HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

	//wait until it is possible to write
	//while(!dataAvailable);
	HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);
	int bufferSize=(slave_header[2]<<8|slave_header[1]);
	if(bufferSize>=size){
		HAL_SPI_Transmit(&hspi3,command,size,1);
		result=0;
	}else{
		result=-1;
	}
	//HAL_GPIO_WritePin(CPU_LED_GPIO_Port,CPU_LED_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
	dataAvailable=0;
	}while(result!=0);

}

void catchBLE(){

int result=fetchBleEvent(buffer,127);
	  if(result==BLE_OK){
		  if(checkEventResp(buffer,EVENT_DISCONNECTED,3)==BLE_OK){
			  setConnectable();
		  }


	  }else{
		 // HAL_GPIO_TogglePin(CHG_LED2_GPIO_Port,CHG_LED2_Pin);
		  //something bad is happening if I am here
	  }




}

void setConnectable(){
	   uint8_t* rxEvent;
	   //Start advertising
	   uint8_t *localname;
	   int res;
	   localname=(uint8_t*)malloc(sizeof(deviceName)+5);//carattere di terminazione+listauid+slavetemp
	   memcpy(localname,deviceName,sizeof(deviceName));
	   localname[sizeof(deviceName)+1]=0x00;
	   localname[sizeof(deviceName)+2]=0x00;
	   localname[sizeof(deviceName)+3]=0x00;
	   localname[sizeof(deviceName)+4]=0x00;
	   localname[sizeof(deviceName)]=0x00;


	   ACI_GAP_SET_DISCOVERABLE[11]=sizeof(deviceName)+1;
	   ACI_GAP_SET_DISCOVERABLE[3]=sizeof(deviceName)+5+sizeof(ACI_GAP_SET_DISCOVERABLE)-4;

	   uint8_t *discoverableCommand;
	   discoverableCommand=(uint8_t*)malloc(sizeof(ACI_GAP_SET_DISCOVERABLE)+sizeof(deviceName)+5);
	   memcpy(discoverableCommand,ACI_GAP_SET_DISCOVERABLE,sizeof(ACI_GAP_SET_DISCOVERABLE));
	   memcpy(discoverableCommand+sizeof(ACI_GAP_SET_DISCOVERABLE),localname,sizeof(deviceName)+5);

	   sendCommand(discoverableCommand,sizeof(deviceName)+5+sizeof(ACI_GAP_SET_DISCOVERABLE));
	   rxEvent=(uint8_t*)malloc(7);
	   while(!dataAvailable);
	   res=fetchBleEvent(rxEvent,7);
	   if(res==BLE_OK){
	   res=checkEventResp(rxEvent,ACI_GAP_SET_DISCOVERABLE_COMPLETE,7);
	   if(res==BLE_OK){
		   stackInitCompleteFlag|=0x80;
	   }
	   }

	   free(rxEvent);
	   free(discoverableCommand);
	   free(localname);
	   HAL_Delay(10);
}



void updateSignedMillesimal(uint8_t *service, uint8_t*characteristic,uint8_t *defaultValue,uint8_t offset, int16_t data){
    uint8_t *newstring;
    newstring=(uint8_t*)malloc(offset+7);
    memcpy(newstring,defaultValue,offset);


	//aumentare di 1 anche per il } e unire alla stringa la nuova stringa
	int flagEmpty=0;
	uint8_t numberInChar[7];
	if(data<0){
    numberInChar[0]=45;
    data=-data;
	}else{
	numberInChar[0]=43;
	}

	numberInChar[1]=data/1000;
    numberInChar[2]=(data-numberInChar[1]*1000)/100;
    numberInChar[3]=(data-numberInChar[2]*100-numberInChar[1]*1000)/10;
    numberInChar[4]=(data-numberInChar[2]*100-numberInChar[3]*10-numberInChar[1]*1000);


    if(numberInChar[1]==0){
    	flagEmpty++;

    	if(numberInChar[2]==0){
    		flagEmpty++;
        	if(numberInChar[3]==0){
        		flagEmpty++;
        	}
    	}
    }



    switch(flagEmpty){
    case 0:{
        numberInChar[1]+='0';
    	numberInChar[2]+='0';
    	numberInChar[3]+='0';
    	numberInChar[4]+='0';
    	numberInChar[5]='\"';
    	numberInChar[6]='}';
    }break;
    case 1:{
        numberInChar[1]='0'+numberInChar[2];
    	numberInChar[2]='0'+numberInChar[3];
    	numberInChar[3]='0'+numberInChar[4];
    	numberInChar[4]='\"';
    	numberInChar[5]='}';
    	numberInChar[6]=' ';


    }break;
    case 2:{
        numberInChar[1]='0'+numberInChar[3];
    	numberInChar[2]='0'+numberInChar[4];
    	numberInChar[3]='\"';
    	numberInChar[4]='}';
    	numberInChar[5]=' ';
    	numberInChar[6]=' ';

    }break;
    case 3:{
        numberInChar[1]='0'+numberInChar[4];
    	numberInChar[2]='\"';
    	numberInChar[3]='}';
    	numberInChar[4]=' ';
    	numberInChar[5]=' ';
    	numberInChar[6]=' ';

    }break;


    default:{
        numberInChar[1]+='0';
    	numberInChar[2]+='0';
    	numberInChar[3]+='0';
    	numberInChar[4]+='0';
    	numberInChar[5]='\"';
    	numberInChar[6]='}';

    }

    }

    memcpy(newstring+offset,numberInChar,7-flagEmpty);


    //non 7 ma dipende
	updateCharValue(service, characteristic, 0,offset+7-flagEmpty, newstring);
	free(newstring);
}

void updateSignedFloat(uint8_t *service, uint8_t*characteristic,uint8_t *defaultValue,uint8_t offset, float data){
	 uint8_t *newstring;
	 newstring=(uint8_t*)malloc(offset+8);
	 memcpy(newstring,defaultValue,offset);


    int16_t newdata=(int16_t)(data*10);
	uint8_t numberInChar[8];
	int flagEmpty=0;
	if(newdata<0){
    numberInChar[0]=45;
    newdata=-newdata;
	}else{
	numberInChar[0]=43;
	}

	numberInChar[1]=newdata/1000;
    numberInChar[2]=(newdata-numberInChar[1]*1000)/100;
    numberInChar[3]=(newdata-numberInChar[2]*100-numberInChar[1]*1000)/10;
    numberInChar[5]=(newdata-numberInChar[2]*100-numberInChar[3]*10-numberInChar[1]*1000);

    if(numberInChar[1]==0){
    	flagEmpty++;

    	if(numberInChar[2]==0){
    		flagEmpty++;
    	}
    }

    switch(flagEmpty){
    case 0:{
        numberInChar[1]+='0';
    	numberInChar[2]+='0';
    	numberInChar[3]+='0';
    	numberInChar[4]='.';
    	numberInChar[5]+='0';
    	numberInChar[6]='\"';
    	numberInChar[7]='}';
    }break;
    case 1:{
        numberInChar[1]='0'+numberInChar[2];
    	numberInChar[2]='0'+numberInChar[3];
    	numberInChar[3]='.';
    	numberInChar[4]='0'+numberInChar[5];
    	numberInChar[5]='\"';
    	numberInChar[6]='}';
    	numberInChar[7]=' ';

    }break;
    case 2:{
        numberInChar[1]='0'+numberInChar[3];
    	numberInChar[2]='.';
    	numberInChar[3]='0'+numberInChar[5];
    	numberInChar[4]='\"';
    	numberInChar[5]='}';
    	numberInChar[6]=' ';
    	numberInChar[7]=' ';

    }break;
    default:{
        numberInChar[1]+='0';
    	numberInChar[2]+='0';
    	numberInChar[3]+='0';
    	numberInChar[4]='.';
    	numberInChar[5]+='0';
    	numberInChar[6]='\"';
    	numberInChar[7]='}';
    }

    }



    memcpy(newstring+offset,numberInChar,8-flagEmpty);


	updateCharValue(service, characteristic, 0, offset+8-flagEmpty, newstring);
	free(newstring);
}




int BLE_command(uint8_t* command, int size, uint8_t* result, int sizeRes, int returnHandles){
	   int response;

	   sendCommand(command,size);
	   rxEvent=(uint8_t*)malloc(sizeRes+2*returnHandles);

	   long contatore=0;
	   while(!HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
		   contatore++;
		   if(contatore>30000){
			   break;
		   }
	   }


	   response=fetchBleEvent(rxEvent,sizeRes+returnHandles*2);
	   if(response==BLE_OK){
		   response=checkEventResp(rxEvent,result,sizeRes);
	   }
	   HAL_Delay(10);


	return response;
}

void addService(uint8_t* UUID, uint8_t* handle, int attributes){


	//memcpy
	memcpy(ADD_CUSTOM_SERVICE+5,UUID,16);
    ADD_CUSTOM_SERVICE[22]=attributes;
	   if(BLE_command(ADD_CUSTOM_SERVICE,sizeof(ADD_CUSTOM_SERVICE),ADD_CUSTOM_SERVICE_COMPLETE,sizeof(ADD_CUSTOM_SERVICE_COMPLETE),1)==BLE_OK){
		   handle[0]=rxEvent[7];
		   handle[1]=rxEvent[8];
	    }
	   free(rxEvent);



}

void addCharacteristic(uint8_t* UUID,uint8_t* handleChar, uint8_t* handleService, uint8_t maxsize, uint8_t proprieties){



	memcpy(ADD_CUSTOM_CHAR+7,UUID,16);


	   ADD_CUSTOM_CHAR[4]= handleService[0];
	   ADD_CUSTOM_CHAR[5]= handleService[1];
	   ADD_CUSTOM_CHAR[23]= maxsize;
	   ADD_CUSTOM_CHAR[25]= proprieties;
	   if(BLE_command(ADD_CUSTOM_CHAR,sizeof(ADD_CUSTOM_CHAR),ADD_CUSTOM_CHAR_COMPLETE,sizeof(ADD_CUSTOM_CHAR_COMPLETE),1)==BLE_OK){
		   handleChar[0]=rxEvent[7];
		   handleChar[1]=rxEvent[8];
	    }
	   free(rxEvent);






}

void updateCharValue(uint8_t* handleService,uint8_t* handleChar, int offset, int size,uint8_t* data){

	UPDATE_CHAR[3]=size+6;
	UPDATE_CHAR[4]=handleService[0];
	UPDATE_CHAR[5]=handleService[1];
	UPDATE_CHAR[6]=handleChar[0];
	UPDATE_CHAR[7]=handleChar[1];
	UPDATE_CHAR[8]=offset;
	UPDATE_CHAR[9]=size;

	uint8_t* commandComplete;
	commandComplete=(uint8_t*)malloc(10+size);
	memcpy(commandComplete,UPDATE_CHAR,10);
	memcpy(commandComplete+10,data,size);

	BLE_command(commandComplete,10+size,ADD_CUSTOM_CHAR_COMPLETE,sizeof(ADD_CUSTOM_CHAR_COMPLETE),0);

	free(commandComplete);
	free(rxEvent);

}


//dopo la disconnessione impostiamo riconnectable e poi fine
