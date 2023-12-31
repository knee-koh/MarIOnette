<img src="Media/Photos/LogoTransparent.png" width=100% />
<p float="left" align="center">
	<img src="Media/Gifs/SpidahBahtVid.gif" width=45% />
	<img src="Media/Gifs/DrawBotV2.gif" width=45% />
</p>

MarIOnette is a Blender plugin for controlling Arduino-based microcontrollers over Serial

[See it in action here](https://www.youtube.com/watch?v=sCcKe_KH84M)

MarIOnette has been tested on most AVR-Based Arduino-compatible microcontrollers. ARM-Based microcontrollers (Teensy) have also been verified. Other microcontrollers (like ESP32) have not yet been tested.

### A few examples and templates have been prepared to get you up and running (see the Examples folder)
[Post-it Drawing Robot](https://www.youtube.com/watch?v=nJkThBeOZog)

A more general video describing the interface, mixing motors + LEDs, and gotchas is coming soon!


### Requirements
- Blender 3.0 or above
- Arduino 2.0 (Earlier versions have also been tested to work)
    - Install the following libraries for Arduino:
        - AccelStepper
        - PWMServo
        - Adafruit_Neopixel
        - Dynamixel2Arduino

### Installation on Mac
1. Download the Marionette.zip file from the Blender Plugin folder
2. Open Blender
3. Go to edit > preferences
4. Select "Add Ons"
5. Click on "Install" in the upper right
6. A browser should pop up. Navigate to the .zip file you downloaded and select it
7. Click on the check box next to the add-on to enable it
8. To view MarIOnette, press 'N' to bring up the side bar

### Installation on Windows
1. Download the Marionette.zip file from the Blender Plugin folder
2. (IMPORTANT) Open Blender as Administrator
3. Go to edit > preferences
4. Select "Add Ons"
5. Click on "Install" in the upper right
6. A browser should pop up. Navigate to the .zip file you downloaded and select it
7. Click on the check box next to the add-on to enable it
8. To view MarIOnette, press 'N' to bring up the side bar

### Setup
1. Add actuators inside the actuator panel
2. Add leds inside the LEDs panel
3. Make sure you select all the correct pins as they are attached to your microcontroller
4. In the sync panel, name your project and select a save directory
5. Hit "Sync"
6. Open your file browser and navigate to the project you just created
7. Open the MarIOnette_Template_V1.ino file; Arduino should launch
8. Plug in your microcontroller
9. Specify your board and serial port
10. Hit "Upload"
11. Back inside Blender, Enable Serial in the MarIOnette Serial panel
12. Refresh the serial ports and select the desired port
13. Press "Connect"
14. Go through your actuators and adjust the mapping values until the viewport matches what's on your desk
    - For bones: 
       1. Move the bone to the minimum angle value (-90 by default)
       2. Change the minimum mapped value (1000 by default) until you have a match
       3. Move the bone to the maximum angle value (90 by default)
       4. Change the maximum mapped value (2000 by default) until you have a match
       5. Repeat for all actuators


### IMPORTANT
For MarIOnette to send values to the microcontroller, you must keep the MarIOnette tab open and visible in your Blender viewport

Dynamixels and Bus Servos have only been tested on Teensy microcontrollers. Because MarIOnette uses the main Serial port on the microcontroller, another serial port (Serial1, Serial2, etc.) is used to communicate with the Dynamixels or Bus Servos. If you want to use MarIOnette on an Arduino Uno (or other microcontroller with only one Serial Port), you will have to modify the Arduino code to use the SoftwareSerial library. Please look at the examples in the Dynamixel2Arduino library for guidance.

Dynamixels and Bus Servos need an additional board to convert the RX and TX lines to a TTL Half-Duplex single wire. LewanSoul and Dynamixel should have these boards available on their websites/Amazon. You can also create your own board for the Dynamixel by following their wiring diagram. (I have a prototype PCB working for Dynamixel modules)[https://github.com/knee-koh/UARTtoTTL], though it can definitely be made smaller with other components.

## MarIOnette currently supports the following actuators:
<pre>                      Servos                                          PWM</pre>
<p float="left" align="center">
	<img src="Media/Gifs/DrawBotV1.gif" width=45% />
	<img src="Media/Gifs/PiroIrisSmoke.gif" width=45% />
</p>

<pre>                  ON/OFF(Solenoids)                          Dynamixels and Bus Servos</pre>
<p float="left" align="center">
	<img src="Media/Gifs/Piro1.gif" width=45% />
	<img src="Media/Gifs/SpidahBahtVid.gif" width=45% />
</p>

### And LEDs:
<pre>                  Neopixels</pre>
<img src="Media/Gifs/LEDTEST.gif" width=50%>

Support for more actuators and features coming soon...

<img src="Media/Gifs/ElmoInput1.gif" width=50%>

Have Fun!
