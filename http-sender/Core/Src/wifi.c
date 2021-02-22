/*
 * wifi.c
 *
 *  Created on: 19 feb 2021
 *      Author: Raul Rosa
 */


#include "wifi.h"
#include "config.h"
#include "main.h"


extern SPI_HandleTypeDef hspi3;


int flagData=0;
int flagReady=0;
int wifiReady=0;
int flagConnected=0;
int flagConnectedFinal=0;

uint8_t buffer[1000];
uint8_t bufferBig[1000];
uint8_t dummy[]={0x0a,0x0a};
uint8_t ready[]={0x15,0x15,0x0a,0x0d,0x20,0x3e};//ready buffer
int index=0;
uint8_t commandbuffer[500];

int indexdata=0;
int indexsensor=0;

uint8_t payloadsensors[1000];






int checkWifiEvent(uint8_t * input, uint8_t *event, int size){

	int i=0;
	for(i=0;i<size;i++){
    if(input[i]!=event[i]){
    	return 0;
    }


	}
	return 1;

}


int getWifiModuleReady(){

  while(!flagData);
  flagData=0;
  memset(buffer,0,sizeof(buffer));
  HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
  do{

	  HAL_SPI_TransmitReceive(&hspi3,dummy,buffer+index,2,1);
	  index+=2;

  }while(HAL_GPIO_ReadPin(WIFI_INT_GPIO_Port,WIFI_INT_Pin)==1);
  HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);

  if(checkWifiEvent(buffer,ready,sizeof(ready))){
	  return 1;
  }
  index=0;
  return 0;

}

void sendCommand(uint8_t *command,int size){

	while(!flagData);
	flagData=0;
    index=0;
	memset(buffer,0,sizeof(buffer));
	memset(commandbuffer,0x0a,sizeof(commandbuffer));
	bigToLittle(command,commandbuffer,size);
	index=0;
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	do{

	HAL_SPI_TransmitReceive(&hspi3,commandbuffer+index,buffer+index,2,1);
	index+=2;

	}while(index<size);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
	while(!flagData);
	flagData=0;
	index=0;
	memset(buffer,0,sizeof(buffer));
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	do{

	HAL_SPI_TransmitReceive(&hspi3,dummy,buffer+index,2,1);
	index+=2;

	}while(HAL_GPIO_ReadPin(WIFI_INT_GPIO_Port,WIFI_INT_Pin)==1);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
	memset(bufferBig,0,sizeof(bufferBig));
	littleToBig(buffer,bufferBig,sizeof(buffer));
	HAL_Delay(1);

}




void littleToBig(uint8_t *input, uint8_t *output, int size){

	int temp,i;
	for(i=0;i<size;i+=2){
		output[i]=input[i+1];
		output[i+1]=input[i];

	}


}

void bigToLittle(uint8_t *input, uint8_t *output, int size){

	int temp,i;
	for(i=0;i<size;i+=2){
		output[i]=input[i+1];
		output[i+1]=input[i];

	}


}

void setSensor(uint8_t *name){
	uint8_t value[50];
	if(indexsensor!=0){
	strcat(payloadsensors,"} ] },{ \"sensor\":");
	strcat(payloadsensors,"\"");
	strcat(payloadsensors,name);
	strcat(payloadsensors,"\"");
	strcat(payloadsensors,", \"data\": [ {");
	}else{
	memset(payloadsensors,0,sizeof(payloadsensors));
	strcpy(payloadsensors,"\"");
	strcat(payloadsensors,name);
	strcat(payloadsensors,"\"");
	strcat(payloadsensors,", \"data\": [ {");
	}
	indexsensor++;
	indexdata=0;

}

void setDataInt(uint8_t *payload, int data){
	uint8_t value[10];
	if(indexdata!=0){
		sprintf(value,",");
		strcat(payloadsensors,value);
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,payload);
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,":");
		sprintf(value,"\"%d\"",data);
		strcat(payloadsensors,value);
	}else{
		//memset(payloadsensors,0,sizeof(payloadsensors));
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,payload);
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,":");
		sprintf(value,"\"%d\"",data);
		strcat(payloadsensors,value);

	}
	indexdata++;


}

void setDataFloat(uint8_t *payload, float data){
	uint8_t value[10];
	if(indexdata!=0){
		sprintf(value,",");
		strcat(payloadsensors,value);
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,payload);
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,":");
		sprintf(value,"\"%.2f\"",data);
		strcat(payloadsensors,value);
	}else{
		//memset(payloadsensors,0,sizeof(payloadsensors));
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,payload);
		strcat(payloadsensors,"\"");
		strcat(payloadsensors,":");
		sprintf(value,"\"%.2f\"",data);
		strcat(payloadsensors,value);

	}
	indexdata++;


}

void sendData(){
	 sendCommand((uint8_t*)STARTTCPcommand, strlen(STARTTCPcommand));


	 int payloadsize=strlen(POSTmid)+strlen(payloadsensors)+strlen(POSTclose)-4;
	  uint8_t DATAcommand[1000];
	  uint8_t DATAsend[1000];
	  memset(DATAcommand,0,sizeof(DATAcommand));
	  memset(DATAsend,0,sizeof(DATAsend));
	  strcpy(DATAcommand,POSTinit);
	  uint8_t data[10];
	  memset(data,0,sizeof(data));
	  sprintf(data,"%d",payloadsize);
	  strcat(DATAcommand,data);
	  strcat(DATAcommand,POSTmid);
	  strcat(DATAcommand,payloadsensors);
	  strcat(DATAcommand,POSTclose);
	  strcpy(DATAsend,TCPSENDcommand);
	  memset(data,0,sizeof(data));
	  sprintf(data,"%d",strlen(DATAcommand)-1);
	  strcat(DATAsend,data);
	  strcat(DATAsend,DATAcommand);
	  sendCommand(DATAsend,strlen(DATAsend));
	  sendCommand((uint8_t*)CLOSETCPcommand, strlen(CLOSETCPcommand));
	  HAL_Delay(TIME_INTERVAL);
	  indexdata=0;
	  indexsensor=0;
}

void startWifi(){
	  //let's set the wifi settings
	  sendCommand((uint8_t*) SSIDcommand,strlen(SSIDcommand));
	  sendCommand((uint8_t*) PSWcommand,strlen(PSWcommand));
	  sendCommand((uint8_t*) SECcommad,strlen(SECcommad));
	  sendCommand((uint8_t*) CONNECTcommand,strlen(CONNECTcommand));
	  sendCommand((uint8_t*)TCPcommand,strlen(TCPcommand));
	  sendCommand((uint8_t*)IPcommand, strlen(IPcommand));
	  sendCommand((uint8_t*)PORTcommand, strlen(PORTcommand));
}
