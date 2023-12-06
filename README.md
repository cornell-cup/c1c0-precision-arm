# c1c0-precision-arm
Code to control precision arm on c1c0.  

## PreciseArmMain
Code to control precise arm

## PreciseArmNoJetson
Code to control precise arm without jetson communication. Used to make sure motors are working.

## blue_arm_main
Code to control blue arm if precise arm is unavailable. Should receive commands same way as precision arm.

## libs/MovingEncoderLib
Library to control encoders placed on precise arm over SPI

## libs/MovingSteppersLib
Library to control stepper motors on precise arm. We don't really use this anymore.


## Notes: 
Place arduino code in a folder that has the same name as the arduino file.

Wiring Tip:
1. Every stepper object takes in three pin inputs: step pin, direction pin, encoder pin
2. Driver Wiring reference:
  powered with 24V
  PUL+ and DIR+ are connected to arduino +5V
  PUL- and DIR- are connected to control signal
3. Encoder wiring reference:
  blue: sclock (52)
  blue white: MOSI (51)
  green: GND (arduino)
  green white: 5V (arduino)
  orange: MISO (50)
  orange white: Chip Select
4. Steppers: 
  J1, J2, J3, J4:
    A+: black
    A-: green
    B+: red
    B-: blue
  J5:
    A+: red
    A-: grey
    B+: yellow
    B-: green
  J6:
    A+: red
    A-: green
    B+: yellow
    B-: blue

Direction Pin: 
  For J3: When dirPin is HIGH, motor rotates counterclockwise when looking down at shaft.


