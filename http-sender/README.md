# STM32L4 Discovery kit IoT B-L475E-IOT01A

STM32 is a family of 32-bit microcontroller integrated circuits by STMicroelectronics. Based on ARM® Cortex®-M, STM32 is suitable for low power, real time applications.

The [B-L475E-IOT01A](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) (STM32L4 Discovery kit IoT node) allows users to develop applications with direct connection to cloud servers.
The Discovery kit enables a wide diversity of applications by exploiting low-power communication like BLE.
The support for Arduino Uno V3 and PMOD connectivity provides unlimited expansion capabilities with a large choice of specialized add-on boards.

This Repository contains the source code and the steps to follow to be able to make STM32 read sensor data and send it, in an organized way, to the Tangle (DLT) of the IOTA Network through the Streams layer.


## Sensors
B-L475E-IOT01A has some ST **MEMS** (Micro Electro-Mechanical Systems) and sensors on board such as:

* 6-axis inertial measurement unit (LSM6DSL)
* 3-axis magnetometer (LIS3MDL)
* Altimeter / pressure sensor (LPS22HH)
* Microphone / audio sensor (MP34DT01)
* Humidity and temperature sensor (HTS221)
* ToF sensor to measure distance up to 2 meters (VL53L0X)

# Download the firmware on the board
## Install official IDE ST-Microelectronic
The source code can be built and flashed using the official IDE of St-microelectronics, supported IDEs are:

* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) available for Linux, Mac and Windows
* [SW4STM32](https://www.st.com/en/development-tools/sw4stm32.html) available for Linux, Mac and Windows

## Open the project

Clone the repository

```
git clone https://github.com/iot2tangle/STM32_B-L475E-IOT01A.git
cd STM32_B-L475E-IOT01A/http-sender

```


Once the installation of the IDE is complete click on *.cproject* file if you installed SW4STM32, the IDE will launch importing the project.

If you installed STM32CubeIDE click on the *.project* file and the IDE will import the project.

## Network configuration

The *config.h* file must be opened and modified, this file is in the directory *'STM32_B-L475E-IOT01A/http-sender/Core/Inc/'* of the repository.

The parameters that MUST be changed are:


```
#define SSID "YOURWIFI"            //name of the wifi
#define PSW "YOURPASSWORD"         //Wifi Password
#define IPgw "192.168.43.201"     //Ip address of the gateway
#define PORTgw "8080"            //Port of the gateway
#define deviceid "456"           //the whitelist of the device in the gateway
#define TIME_INTERVAL 1000       //time in ms between each data sent to gateway

```

## Flash the code

Connect the board to your pc using the (USB ST-link) connector, click on *run* on the IDE and the project will be compiled and flashed on the board. Press the reset button on the board and the device will start to work. 

# Setting up the Streams HTTP Gateway

## Preparation

Install Rust if you don't have it already. More info about Rust here https://www.rust-lang.org/tools/install

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Make sure you also have the build dependencies installed, if not run:  

```
sudo apt update
sudo apt install build-essential pkg-config libssl-dev  
```

## Installing the Streams Gateway
Get the Streams Gateway repository
https://github.com/iot2tangle/Streams-http-gateway

```
git clone https://github.com/iot2tangle/Streams-http-gateway
```

Navigate to the root of **Streams-http-gateway** directory and edit the **config.json** file to define yours *device names*, *endpoint*, *port*, you can also change the IOTA Full Node used, among others.

## Start the Streams Server

### Sending messages to the Tangle

Run the Streams Gateway:  

```
cargo run --release  
```
This will compile and start the *Streams HTTP Gateway*. Note that the compilation process may take from 3 to 25 minutes (Pi3 took us around 15/25 mins, Pi4 8 mins and VPS or desktop machines will generally compile under the 5 mins) depending on the device you are using as host.
You will only go through the compilation process once and any restart done later will take a few seconds to have the Gateway working.

Once started, the ***Channel Id*** will be displayed, and the gateway will be open waiting for data to send to the Tangle.

![Streams Gateway receiving data](https://i.postimg.cc/zfz0tbWz/Screenshot-from-2020-10-16-11-44-59.png)
*The Channel Id that will allow subscribers to access the channel data.*

### Reading messages from the Tangle

You can read the received messages directly from the **I2T Explorer**: https://explorer.iot2tangle.io/ using the Channel Id printed by the Gateway in shell.   

![I2T Explorer](https://i.postimg.cc/wTNf7dgp/Screenshot-from-2020-10-16-11-46-16.png)


*For inquiries, improvements or errors detected, please start an issue in this repository.*
