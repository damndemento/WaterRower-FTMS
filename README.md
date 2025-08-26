# WaterRower-FTMS for my Vendomnia Rower

This project is based on the **ArduRower** project from **zpukr (https://github.com/zpukr/ArduRower)**, thank you very much for your support!

I build this for my **Vendomnia Rower** to add BLE / FTMS support to the rower. I used a cheap **esp32lolin clone** for this project.
It should work with **Coxswain** on Android devices and with other apps that support the **FTMS** (FiTness Machine Service) protocol.

I added a second sensor to the rower and changed the logic for strokes and acceleration accordingly.

There is a precompiled firmware in the firmware folder of this project. I used VSCode with **PlatformIO** to edit and compile this project.

Use the precompiled "firmware_1magnet.bin" or download and import the project to VSCode or similar and compile and upload your own firmware.bin file to the device.

![alt tag](https://github.com/damndemento/vendomniaBLE/blob/main/wemos-esp32_com_oled-pinout.webp)

![alt tag](https://github.com/damndemento/vendomniaBLE/blob/main/esp32lolin.jpg)
