/* 

MarIOnette template file for Arduino-compatible boards

Tested on Arduino Nano (ATMega 328P), Arduino Uno, Teensy 3.2, Teensy 3.6, and Teensy 4.1

*/

#define DEBUG_SETUP 1

#include "config.h"
#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h> // Support for other indexable LEDs coming soon...

// For AVR - based boards without a dedicated Serial buffer, we need to add a small delay between Serial reads
#if defined(__AVR__)
  #define PARSE_INT 1
#else
  #define PARSE_INT 0
#endif

/*

If using Neopixels, we must use the PWMServo library. This is due to the Neopixel library disabling interrupts, which
the Servo library depends on. This reduces the usable servo range to 0-180 (from the larger range of 700-2300).

You can avoid this issue by using a dedicated PWM I2C board like the PCA9685 16 Channel PWM Servo Motor Driver.
Alternatively, you can switch to indexable LEDs with a dedicated clock pin (APA102 or SK9822).

Support for both of these is coming some time in the future (whenever I get some free time to work on this).

*/
#if TOTAL_LEDS > 0
  #include "PWMServo.h"
#else
  #include <Servo.h>
#endif

// Interval timer for more precise timing (useful for stepper motors since they need a lot of pulses)
//IntervalTimer myTimer;

// Servos
#if TOTAL_SERVOS > 0 && TOTAL_LEDS > 0
  PWMServo servos[TOTAL_MOTORS];
#elif TOTAL_SERVOS > 0 && TOTAL_LEDS == 0
  Servo servos[TOTAL_MOTORS];
#endif

// Steppers
#if TOTAL_STEPPERS > 0
  AccelStepper steppers[TOTAL_STEPPERS];
#else
  AccelStepper steppers[1];
#endif

// Neopixels
#if TOTAL_LEDS > 0
  Adafruit_NeoPixel neopixels[TOTAL_LEDS];
#else
  Adafruit_NeoPixel neopixels[1];
#endif



// MarIOnette serial
unsigned int counter = 0;
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

      if(PARSE_INT){
        delay(1);
      }

      // Read in first bytes to get message length
      char one = Serial.read();
      
      if(PARSE_INT){
        delay(1);
      }

      char two = Serial.read();
      howManyBytes = word(one, two) + expectedSpeedBytes;
      
      if(debugOut){
        Serial.print("Expecting bytes: ");
        Serial.println(howManyBytes);
      }

      char tempBuffer[howManyBytes];

      while(Serial.available()){
        if(PARSE_INT){
          delay(1);
        }

        inByte = Serial.read();
        
        // Keep reading in case last byte is the stop char
        if(counter < howManyBytes){
          tempBuffer[counter] = inByte;

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
              int j = 0;
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

              else if(motor_values[i][0] == 5){
                steppers[j].moveTo(word(tempBuffer[i*2+expectedSpeedBytes], tempBuffer[i*2+expectedSpeedBytes+1]));
                j++;
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
        }
      }
    }
  }
}

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

void setupMotors(){
  for(int i = 0; i < TOTAL_MOTORS; i++){
    // Servos
    if(motor_values[i][0] == 1){
      servos[i].attach(motor_values[i][1]);
    }

    // PWM pin
    else if(motor_values[i][0] == 2){
      pinMode(motor_values[i][1], OUTPUT);
    }
  }
}

void setupSteppers(){
  for(int i = 0; i < TOTAL_STEPPERS; i++){
    for(int j = 0; j < TOTAL_MOTORS; j++){
      if(motor_values[j][0] == 5){
        steppers[i] = AccelStepper(steppers[i].DRIVER, motor_values[j][1], motor_values[j][2]);

        steppers[i].setMaxSpeed(motor_values[j][4]); // 100mm/s @ 80 steps/mm
        steppers[i].setAcceleration(motor_values[j][5]); // 2000mm/s^2
        steppers[i].setPinsInverted(false, false, true);
        steppers[i].enableOutputs();
        i++;

        if(DEBUG_SETUP){
          Serial.print("Stepper number ");
          Serial.print(i+1);
          Serial.print(" Step Pin: ");
          Serial.print(motor_values[j][1]);
          Serial.print(" Dir Pin: ");
          Serial.print(motor_values[j][2]);
          Serial.print(" Speed: ");
          Serial.print(motor_values[j][4]);
          Serial.print(" Acceleration: ");
          Serial.println(motor_values[j][5]);
        }
      }
    }
  }
}

void setup() {
  // Start Serial monitor
  Serial.begin(BAUD_RATE);
  Serial.println("Starting MarIOnette initialization...");

  // Servo and motor init
  if(TOTAL_MOTORS > 0){
    setupMotors();
    Serial.println("Servo init done");
  }

  // Neopixel
  if(TOTAL_LEDS > 0){
    setupLEDs();
    Serial.println("Neopixel ring init done");
  }

  // Stepper motor initialization
  setupSteppers();
  
  //myTimer.begin(stepper_update, 15);
  //myTimer.priority(0);

  Serial.println("Setup finished!");
}

void loop() {
  // Begin reading serial port for incoming commands
  readSerialBytes();

  if(TOTAL_STEPPERS > 0){
    for(int i = 0; i < TOTAL_STEPPERS; i++){
      steppers[i].run();
    }
  }
}