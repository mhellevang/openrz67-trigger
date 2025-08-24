# openrz67-trigger

openrz67-trigger is a arduino compatible project made for enabling remote triggering of the Mamiya RZ67 analog camera from an app. openrz67-trigger is made to run on a small ESP32 device (in this case Seeed XIAO ESP32C3) that is connected to the electrical input pins on the Mamiya RZ67 camera.

There is also an android app made for communication with openrz67-trigger, see [openrz67-android](https://github.com/mhellevang/openrz67-android).

# What is the Mamiya RZ67?

The [Mamiya RZ67](https://en.wikipedia.org/wiki/Mamiya_RZ67) is a great analog medium format film camera by Mamiya,
first made in 1982. It's the pinnacle of medium format studio cameras. 

![Photo of Mamiya RZ67](assets/Mamiya_RZ67_Professional_and_a_Fujifilm_color_120_format_roll_film_(60_mm_wide).jpg)
*Photo by Thomas Claveirole from Paris, France, CC BY-SA 2.0 <https://creativecommons.org/licenses/by-sa/2.0>, via Wikimedia Commons*

# How do I get my own device?

This project is still very much a work in progress. 

I'll publish my recommendations for making your own device once I've made a prototype I'm happy with. If you want to start working on making your own device now, you can use the schematics and code in this repository as a starting point, but do expect to make some changes to get it to work for you. Tinkering is half the fun!

## Hardware

* 1x Seeed XIAO ESP32C3 (or similar ESP32 device with BLE) to receive the bluetooth commands and trigger the camera
* 1x IRF520 MOSFET (or similar) to control the shutter release
* Wires
* Battery or other power source (LiPo, USP-C) 

## Camera connection

The camera has a four pin IO port in front. The pins are labeled as follows (left to right):

* 1: 6v. Can be ignored, we don't need it for this project.
* 2: GND (Ground)
* 3: S1 switch
* 4: S2 switch

To trigger the shutter release from the ESP32, we need to connect GND, S1 and S2 via the MOSFET.

## Wiring

![Schematics](assets/RZ67_Seeed_XIAO_ESP32C3_bb.png?)
