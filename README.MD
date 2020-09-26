### Program

1. Download NRF52 SDK 16.0 [link](https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v16.x.x/)
2. Clone repo
3. Fix path to sdk in Makefile:
``SDK_ROOT := ..``
``PROJ_DIR := .``
4. Configure Device (Enable/Disable "Low Bat sleep", relevant when powering the board from SWD):
`` make sdk_config ``
5. Flash Softdevice
`` make flash_softdevice ``
6. Flash App
`` make flash ``
7. Bootloader and Firmware validation
-  `` cd DFU/bootloader ``
-  `` make batch ``

For security reasons, generate a new private/public key in the DFU folder
- ``cd DFU/bootloader``
- ``nrfutil keys generate private_key.pem``
- ``nrfutil keys display --format code --out_file "public_key.c" --key pk private_key.pem``
