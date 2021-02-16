# STM32L4 Discovery kit IoT B-L475E-IOT01A

STM32 is a family of 32-bit microcontroller integrated circuits by STMicroelectronics. Based on ARM® Cortex®-M, STM32 is suitable for low power, real time applications.

The [B-L475E-IOT01A](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) (STM32L4 Discovery kit IoT node) allows users to develop applications with direct connection to cloud servers.
The Discovery kit enables a wide diversity of applications by exploiting low-power communication like BLE.
The support for Arduino Uno V3 and PMOD connectivity provides unlimited expansion capabilities with a large choice of specialized add-on boards.

## On board sensors
B-L475E-IOT01A has some ST **MEMS** (Micro Electro-Mechanical Systems) and sensors on board such as:

* 6-axis inertial measurement unit (LSM6DSL)
* 3-axis magnetometer (LIS3MDL)
* Altimeter / pressure sensor (LPS22HH)
* Microphone / audio sensor (MP34DT01)
* Humidity and temperature sensor (HTS221)
* ToF sensor to measure distance up to 2 meters (VL53L0X)

## Connectivity

The board is capable of connecting the device with different wireless protocols such as:

* BLE
* Wi-Fi
* NFC
* RF 915MHz

# Available connectivity

* [BLE-sender](https://github.com/iot2tangle/STM32_B-L475E-IOT01A/tree/main/BLE-sender) -- B-L475E-IOT01A will send the sensors data using *Bluetooth Low Energy (BLE)* to [I2T BLE Gateway](https://github.com/iot2tangle/Streams-ble-gateway)
