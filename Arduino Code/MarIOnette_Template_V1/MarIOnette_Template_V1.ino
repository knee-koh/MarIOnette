/* MarIOnette demo with 1 stepper, 2 servos, and one 16pixel neopixel ring 

Because of Neopixels killing interrupts, PWMServo must be used, which slows down a timer to 50Hz

Teensy 3.2 has 3 timers that control a number of pins:
FTM0	5, 6, 9, 10, 20, 21, 22, 23	    @488.28 Hz
FTM1	3, 4	                          @488.28 Hz
FTM2	25, 32	                        @488.28 Hz

*/

#include "config.h"
#include <Arduino.h>

#if STEPPERS > 0
  #include <TMCStepper.h>
  #include <AccelStepper.h>
#endif

#include <Servo.h>

//#include <PWMServo.h>
#if TOTAL_LEDS > 0
  #include <Adafruit_NeoPixel.h>
  #include "PWMServo.h"
#else
  #include <Servo.h>
#endif

IntervalTimer myTimer;

//ESP32Encoder encoder;

#define EN_PIN              4 // Enable
#define DIR_PIN             2 // Direction
#define STEP_PIN            3 // Step
#define CS_PIN              15 // Chip select

#define STALL_VALUE         15 // [-64..63]

#define SERVO_1             21
#define SERVO_2             20
#define NEOPIXEL_PIN        10 // Move to a different pin!!! see above

#define R_SENSE             0.11f

#define NUM_PIXELS          16

#define PWM_PIN_1           6 // Move to another pin!!! 3, 4, 25, or 32 see above


// 80 steps per mm for 16 microsteps, 160 for 32, 320 for 64, 640 for 128, 1280 for 256ss

// Stepper Driver Initialization
TMC2130Stepper driver(CS_PIN, R_SENSE);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

uint32_t steps_per_mm = 1280;
uint16_t steps_per_rev = 200;
uint32_t default_microsteps = 128;
uint32_t microsteps_per_rev = steps_per_rev * default_microsteps;
uint32_t stepper_max_speed = 5000000;
uint32_t stepper_max_acceleration = 5000000;

// Servos
#if TOTAL_MOTORS > 0 && TOTAL_LEDS > 0
  PWMServo servos[TOTAL_MOTORS];
#elif TOTAL_MOTORS > 0 && TOTAL_LEDS == 0
  Servo servos[TOTAL_MOTORS];
#endif

// Neopixels
#if TOTAL_LEDS > 0
  Adafruit_NeoPixel neopixels[TOTAL_LEDS];
#else
  Adafruit_NeoPixel neopixels[1];
#endif

//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

// MarIOnette
unsigned int counter;
unsigned int howManyBytes = 0;

// Expected packet
const int expectedMotorBytes = TOTAL_MOTORS * 2;
const int expectedLEDSingleBytes = TOTAL_LEDS * 4; // RGBW for now
const int expectedLEDAllBytes = 16 * 4;
const int expectedSpeedBytes = 2;

// Values
long stepper_value = 0;
long servo1_value = 0;
long servo2_value = 0;
long ledValue_R = 0;
long ledValue_G = 0;
long ledValue_B = 0;

// To save on Serial bandwidth, read in two bytes per motor and combine them into an int
void readSerialBytes(){
	int debugOut = 0;
	char inByte;

	if(Serial.available() > 0){
		char mode = Serial.read();

		// Blender sending data...
		if(mode == 'A'){
			counter = 0;

			// Read in first bytes to get message length
			char one = Serial.read();
      delay(1);
			char two = Serial.read();
			howManyBytes = word(one, two) + expectedSpeedBytes;
			
			if(debugOut){
				Serial.print("Expecting bytes: ");
				Serial.println(howManyBytes);
			}

			char tempBuffer[howManyBytes];

			while(Serial.available()){
				inByte = Serial.read();
        delay(1);
				
				// Keep reading in case last byte is the stop char
				if(counter < howManyBytes){
					tempBuffer[counter] = inByte;

          if(debugOut){
            Serial.print("Byte  ");
            Serial.print(counter);
            Serial.print(" | ");
            Serial.print(inByte);
          }

					counter++;
				}

				// Received too many bytes but no stop character...
				else if(counter >= howManyBytes && inByte != '.'){							
					while(Serial.available()){
						Serial.read();
						counter++;
					}

					if(debugOut){
						Serial.print("Too many bytes, expected ");
						Serial.print(howManyBytes);
						Serial.print(" and got ");
						Serial.println(counter);
					}

					counter = 0;
					break;
				}

				// Right amount of bytes received, set motors and LEDs
				else if(counter == howManyBytes && (inByte == '.' || inByte == 46)){
					if(debugOut){
						Serial.println("Success!");
					}

          if(TOTAL_MOTORS > 0){
            //speed = word(tempBuffer[0], tempBuffer[1]);
            for(int i = 0; i < TOTAL_MOTORS; i++){
              if(motor_values[i][0] == 1){
                if(TOTAL_LEDS > 0){
                  servos[i].write(word(tempBuffer[i*2+expectedSpeedBytes], tempBuffer[i*2+expectedSpeedBytes+1]));
                }

                else{
                  servos[i].writeMicroseconds(word(tempBuffer[i*2+expectedSpeedBytes], tempBuffer[i*2+expectedSpeedBytes+1]));
                }
              }

              else if(motor_values[i][0] == 2){
                analogWrite(motor_values[i][1], word(tempBuffer[i*2+expectedSpeedBytes], tempBuffer[i*2+expectedSpeedBytes+1]));
              }
            }
          }

          if(TOTAL_LEDS > 0){
            // Update each LED independently
            if(howManyBytes > expectedSpeedBytes + expectedMotorBytes + expectedLEDSingleBytes){
              for(int i = 0; i < TOTAL_LEDS; i++){
                long offset = expectedSpeedBytes + expectedMotorBytes;

                for(int j = 0; j < led_values[i][2]; j++){
                  int red = tempBuffer[offset + j*4];
                  int green = tempBuffer[offset + j*4 + 1];
                  int blue = tempBuffer[offset + j*4 + 2];
                  int white = tempBuffer[offset + j*4 + 3];

                  neopixels[i].setPixelColor(j, red, green, blue, white);
                }

                neopixels[i].show();
              }
            }
            

            // Otherwise treat LED strip as a single color
            else{
              long offset = expectedSpeedBytes + expectedMotorBytes;
              for(int i = 0; i < TOTAL_LEDS; i++){
                int red = tempBuffer[offset + i*4];
                int green = tempBuffer[offset + i*4 + 1];;
                int blue = tempBuffer[offset + i*4 + 2];;
                int white = tempBuffer[offset + i*4 + 3];;

                for(int j = 0; j < led_values[i][2]; j++){
                  neopixels[i].setPixelColor(j, red, green, blue, white);
                }

                neopixels[i].show();
              }
            }
          }

          /*
					stepper_value = word(tempBuffer[2], tempBuffer[3]);
					servo1_value = word(tempBuffer[4], tempBuffer[5]);
          servo2_value = word(tempBuffer[6], tempBuffer[7]);
          ledValue_R = word(tempBuffer[8], tempBuffer[9]);
          ledValue_G = word(tempBuffer[10], tempBuffer[11]);
          ledValue_B = word(tempBuffer[12], tempBuffer[13]);
          */
          
					// Update steppers
					//stepper.moveTo(stepper_value);

          // Update servos
          //servo1.write(servo1_value);
          // /servo2.write(servo2_value);
        }
					
        if(debugOut){
          Serial.print("Expected length: ");
          Serial.print(howManyBytes);
          //Serial.print(" Speed: ");
          //Serial.print(speed);
          Serial.print(" M1: ");
          Serial.print(stepper_value);
          Serial.print(" M2: ");
          Serial.println(stepper_value);
        }
			}
		}

		// Spit buffer back out
		else if(mode == 'P'){
			Serial.print("Buffer: ");
			//Serial.println(inputBuffer);
		}
  }
	
	// Received an incomplete packet
	else if(counter > 0){
		if(debugOut){
      Serial.println("Incomplete packet!");
			Serial.println(counter);
		}
		counter = 0;
	}
}

/*
void stepper_update(){
  stepper.run();
}
*/

void debugLeds(int delay_time){
  for(int i = 0; i < 255; i++){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, i, 0, 0, 0);
    }

    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  delay(200);

  for(int i = 254; i >= 0; i--){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, i, 0, 0, 0);
    }

    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  delay(200);

  for(int i = 0; i < 255; i++){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, 0, i, 0, 0);
    }
   
    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  for(int i = 254; i >= 0; i--){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, 0, i, 0, 0);
    }

    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  delay(200);

  for(int i = 0; i < 255; i++){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, 0, 0, i, 0);      
    }
   
    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  for(int i = 254; i >= 0; i--){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, 0, 0, i, 0);
    }

    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  delay(200);

  for(int i = 0; i < 255; i++){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, 0, 0, 0, i);
    }
   
    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  for(int i = 254; i >= 0; i--){
    for(int j = 0; j < NUM_PIXELS; j++){
      neopixels[0].setPixelColor(j, 0, 0, 0, i);
    }

    neopixels[0].show();
    delayMicroseconds(delay_time);
  }

  delay(200);
}

void debugPWM(int delay_time){
  for(int i = 0; i < 50; i++){
    analogWrite(PWM_PIN_1, i);
    delayMicroseconds(delay_time);
  }

  delay(500);

  for(int i = 50; i >= 0; i--){
    analogWrite(PWM_PIN_1, i);
    delayMicroseconds(delay_time);
  }

  delay(1500);
}

/*

void debugServos(int delay_time){
  for(int i = 0; i < 180; i++){
    servo1.write(i);
    servo2.write(i);
    delay(delay_time);
  }

  delay(500);

  for(int i = 180; i > 0; i--){
    servo1.write(i);
    servo2.write(i);
    delay(delay_time);
  }

  delay(500);
}

void debugSteppers(){
  stepper.moveTo(200*default_microsteps);

  while(stepper.isRunning()){
    stepper.run();
  }

  delay(200);

  stepper.moveTo(0*default_microsteps);

  while(stepper.isRunning()){
    stepper.run();
  }

  delay(200);

}
*/

void setupLEDs(){
  for(int i = 0; i < TOTAL_LEDS; i++){
    if(led_values[i][3] == 1){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_RGB + NEO_KHZ800);
    }

    else if(led_values[i][3] == 2){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_GRB + NEO_KHZ800);
    }

    else if(led_values[i][3] == 3){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_RGB + NEO_KHZ400);
    }

    else if(led_values[i][3] == 4){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_GRB + NEO_KHZ400);
    }

    else if(led_values[i][3] == 5){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_RGBW + NEO_KHZ800);
    }

    else if(led_values[i][3] == 6){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_GRBW + NEO_KHZ800);
    }

    else if(led_values[i][3] == 7){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_RGBW + NEO_KHZ400);
    }

    else if(led_values[i][3] == 8){
      neopixels[i] = Adafruit_NeoPixel(led_values[i][2], led_values[i][1], NEO_GRBW + NEO_KHZ400);
    }

    pinMode(led_values[i][1], OUTPUT);
    neopixels[i].begin();
    neopixels[i].clear();
    neopixels[i].show();
    neopixels[i].setBrightness(30);
  }
}

void setup() {
  Serial.begin(1000000);
  Serial.println("Starting drawing bot stepper model V1...");
  //Serial.setTimeout(10);

  // Servos
  //servo1.attach(SERVO_1);
  //servo2.attach(SERVO_2);
  for(int i = 0; i < TOTAL_MOTORS; i++){
    if(motor_values[i][0] == 1){
      servos[i].attach(motor_values[i][1]);
    }

    else if(motor_values[i][0] == 2){
      pinMode(motor_values[i][1], OUTPUT);
    }
  }

  Serial.println("Servo init done");

  // Neopixel
  if(TOTAL_LEDS > 0){
    setupLEDs();
  }

  pinMode(PWM_PIN_1, OUTPUT);
  //analogWriteResolution(10);

  /*
  pinMode(NEOPIXEL_PIN, OUTPUT);
  strip.begin();
  strip.clear();
  strip.show();
  strip.setBrightness(30);
  */
  
  Serial.println("Neopixel ring init done");

  // Stepper motor initialization
  /*
  SPI.begin();

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.println("Stepper pins init done");

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(default_microsteps);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  //driver_L.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop

  stepper.setMaxSpeed(stepper_max_speed); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(stepper_max_acceleration); // 2000mm/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  */

  //myTimer.begin(stepper_update, 15);
  //myTimer.priority(0);
  
  //debugLeds(10);

  Serial.println("Stepper init done");

  Serial.println("Setup finished!");
}

void loop() {
  /*
  debugSteppers();
  debugLEDs(10);
  debugServos(2);
  */

  //debugPWM(10000);
  readSerialBytes();
  
}
