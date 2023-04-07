PICTURE OR GIF OF LOGO

## MarIOnette is a Blender plugin for controlling Arduino-based microcontrollers over Serial

[See it in action here](https://www.youtube.com)

MarIOnette has been tested on most AVR-Based Arduino-compatible microcontrollers. ARM-Based Teensy microcontrollers have also been verified. Other microcontrollers have not yet been tested.

### A few examples and templates have been prepared to get you up and running (see the Examples folder)
[Post-it Drawing Robot](https://www.youtube.com)
[Playground](https://www.youtube.com)

### Requirements
Blender 3.2 or above
Arduino 2.0 (Earlier versions have also been tested to work)
     Install the following libraries:
         AccelStepper
         PWMServo
         Adaftui_Neopixel

### Installation on Mac
Download the Marionette.zip file from the Blender Plugin folder
Open Blender
Go to edit > preferences
Select "Add Ons"
Click on "Install" in the upper right
A browser should pop up. Navigate to the .zip file you downloaded and select it
Click on the check box next to the add-on to enable it
To view MarIOnette, press 'N' to bring up the side bar

### Installation on Windows
Download the Marionette.zip file from the Blender Plugin folder
(IMPORTANT) Open Blender as Administrator
Go to edit > preferences
Select "Add Ons"
Click on "Install" in the upper right
A browser should pop up. Navigate to the .zip file you downloaded and select it
Click on the check box next to the add-on to enable it
To view MarIOnette, press 'N' to bring up the side bar

### Setup
Add actuators inside the actuator panel
Add leds inside the LEDs panel
Make sure you select all the correct pins as they are attached to your microcontroller
In the sync panel, name your project and select a save directory
Hit "Sync"
Open your file browser and navigate to the project you just created
Open the MarIOnette_Template_V1.ino file; Arduino should launch
Plug in your microcontroller
Specify your board and serial port
Hit "Upload"
Back inside Blender, Enable Serial in the MarIOnette Serial panel
Refresh the serial ports and select the desired port
Press "Connect"
Go through your actuators and adjust the mapping values until the viewport matches what's on your desk
  For bones: 
    Move the bone to the minimum angle value (-90 by default)
    Change the minimum mapped value (1000 by default) until you have a match
    Move the bone to the maximum angle value (90 by default)
    Change the maximum mapped value (2000 by default) until you have a match
    Repeat for all actuators


### IMPORTANT
For MarIOnette to send values to the microcontroller, you must keep the MarIOnette tab open and visible in your Blender viewport

### MarIOnette currently supports the following actuators:
Servos

GIFs of examples

PWM - Piro or playground

GIFs of examples

ON/OFF (Solenoid)

GIFs of examples - Piro

Dynamixels and Bus Servos

GIFs of examples - Spidahbot

### And LEDs:
Single channel
Gif of glowing led from playground
Neopixels
Gif of playground or piro example
