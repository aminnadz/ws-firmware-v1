# Wearable Sensor Firmware V1

This repo holds the first version of warable sensor firmware. The state machine that is implemented in this firmware is given in figure below;

![alt text](https://github.com/inovatink/ws-firmware-v1/blob/master/ws-firmware-v1-state-diagram.PNG "State Diagram")

The program reads sensor values periodically and sends them to bluetooth module. RN4871 Bluetooth module is configured in transparent UART mode. In this mode, input (data) present at the serial interface is written to characteristic of a GATT service once the notification for that characteristic is enabled by central BLE device. The sample period and other configurations are done in config.h.

Program starts with MCU initialization and application configuration (in Arduino this is happening in Setup(). Program starts to execute in **Idle** state. Once new data is available from sensor (at nTs) program enters **Sample** state and sensors are sampled. If sampling fails the program returns to **Idle** state and waits for new sample (this rarely happens due to temporary sensor error). After data is read from sensors program enters **Serialize** state where the data from individual sensors is serialized into a message string to be sent via Bluetooth. After serialization program enters **Send** state where data is sent to Bluetooth.

>**IMPORTANT! -- Firmware makes use of RN4871 module pins that may not be initialized. This firmware uses P1_3 pin on module as sleep control pin. Additionally blue led is tied to P1_2 pin which also needs initialization. To initialize module pins run code from ws-simple-BLE-communication to communicate with module through native USB port and execute following commands;**
>```
>- Set Pin P1_3 as UART RX Indication
>SW,0B,04
>- Set Pin P1_2 as Status 2 (connected to LED)
>SW,0A,07
>- Reset Module
>R,1
>```
