# Weather station
esp32-based weather station with variety of sensors

This particular setup requires MQTT, i'm using home-assistant.io and wanted to dig into lower-level mqtt discovery protocols thus manual code. Esphome is a absolutely great if you don't want to deal with low-level stuff yourself.

Arduino code

NB: had two RS-485 devices (https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/Wind_Speed_Transmitter_485Type_V1.0_UG.pdf, https://www.alibaba.com/product-detail/Lightweight-casing-wind-speed-and-direction_62219376461.html) which were using the same RS-485 address. That had to be programmed to NVRAM, contacted manufacturer for documentation.

Register 07D0 H - device address (1-254)
Register 07D1 H - baud speed (0 for 2400 to 6 for 115200)

Python - the script to collect mean values for 5 minutes and upload to windy (their temporal resolution is 5 mins)

