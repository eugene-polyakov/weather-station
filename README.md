# Weather station

Hey folks - the station is situated on the roof of the residential building in Alanya, Turkey (Riverside Resort - https://g.co/kgs/4uqLjyM) at the mouth of a mountain canyon which makes wind reading quite weird in the mornings when the thermals flow down.
Decided to post the data anyways as there's a very poor coverage of the area. This is a home-built custom weather station running on esp32 controller - source code is on the github - https://github.com/eugene-polyakov/weather-station

NB: there are frequent power outages in the area, may be some gaps in reporting.

Namaste !



project details -


esp32-based weather station with variety of sensors

This particular setup requires MQTT as i'm using home-assistant.io and wanted to dig into lower-level mqtt discovery protocols -- thus the manual code. Esphome is a absolutely great if you don't want to deal with low-level stuff yourself.

Arduino code

NB: have two RS-485 devices (https://media.digikey.com/pdf/Data%20Sheets/Seeed%20Technology/Wind_Speed_Transmitter_485Type_V1.0_UG.pdf, https://www.alibaba.com/product-detail/Lightweight-casing-wind-speed-and-direction_62219376461.html) which were using the same RS-485 address. That had to be programmed to NVRAM, contacted manufacturer for documentation.

Register 07D0 H - device address (1-254)

Register 07D1 H - baud speed (0 for 2400 to 6 for 115200)

Python - the script to collect mean values for 5 minutes and upload to windy (their temporal resolution is 5 mins)

