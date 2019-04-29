# ble-enocean_switch_mesh_demo
BLE Enocean switch mesh demo with nRF thingy

This example is to use the nRF52832 DK board as the light client with enocean sniffer and then turn on / off the corresponding thingy.

## Configuration NRF52832 DK
* Button 1: -- toggle thingy #01 LED on/off
* Button 2: -- toggle thingy #02 LED on/off
* Button 3: (odd) -- toggle thingy #01 / #03 on/off
* Button 4: (even) -- toggle thingy #02 / #04 on/off

## Modify the enocean switch address
```
#define ENOCEAN_ADDRESS {0xD4, 0x62, 0x00, 0x00, 0x15, 0xe2};
```

### Project folder
* client_enocean -- project folder of nRF52832 DK light client
* server_thingy -- project folder of the thingy light server

### Location


## Requirement
* nRF52832 DK board
* 4 x Thingy
* IDE : Segger Embedded Studio

To compile it, clone the repository in the .\Nordic_Semiconductor_ASA\nrf5SDKforMeshv310src\ folder. If you download the zip, place each of the project folders of this repository into the .\Nordic_Semiconductor_ASA\nrf5SDKforMeshv310src\examples folder. Please make sure that .\Nordic_Semiconductor_ASA\nRF5_SDK_15.2.0_9412b96 is also present unless you modify the MESH_SDK path inside the SES project.
