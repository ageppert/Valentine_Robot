// Valentine's Day Robot Project
// V1.0 First releease
// V1.1 Replaced standard Servo library with ServoTimer2 library, moved servo to pin 3.
//      Enabled flashing LED eyes during playback
//      Enabled waving sweep during playback, but it is not working.
//      Most likely because audio library disables other interrupts during playback, include servo2
// V1.2 Fixed servo sweep code! No conflict with audio timer. Fixed logic that occurs during playback.

#include <SD.h>              // SD Card library
#include <SPI.h>             // needed for SD card?
#include <TMRpcm.h>          // audio file player, unsigned 8-bit PCM, 8000/11025/16000kHz, mono
                             // uses timer1 (the only 16-bit timer on UNO, using pins PWM 9, 10)
                             // https://github.com/TMRh20/TMRpcm
#include <ServoTimer2.h>     // uses timer2 (8 bit timer, pin 3 and/or 11
                             // https://github.com/jkloo/SWIM-R/tree/master/SW/Arduino/libraries/ServoTimer2

/**** DIGITAL I/O ****/
// Arduino Uno TX line     0
// Arduino Uno RX line     1
#define eye_led2           2  // Output
#define servo1_pin         3  // Output  (Uno Timer2 PWM pin)
#define SD_ChipSelectPin   4  // Output using digital pin 4 on arduino uno
#define eye_led1           5  // Output  (Uno Timer0 PWM pin)
//                         6             (Uno Timer0 PWM pin)
#define sensor1            7  // Input
#define audio_amp_enable   8  // Output enable this pin to turn on the audio amplifier
#define speaker_output     9  // Output  (Uno Timer1 PWM pin)
//                        10  // Output  (Uno Timer1 PWM pin) avoid for servo
// SD Card MOSI           11  // Output Defined in SD Card Library   (Uno Timer2 PWM pin)
// SD Card MISO           12  // Input  Defined in SD Card Library
// SD Card SCK            13  // Output Defined in SD Card Library

/**** ANALOG INPUT ****/
// A0
// A1
// A2
// A3
// A4
// A5

/**** TIMERS       ****/
long previousMillis = 0;        // will store last time LED was updated
long interval = 350;           // interval at which to blink (milliseconds)

/**** VARIABLES ****/
int sensor1State = 0;
int ledState = LOW;             // ledState used to set the LEDs
int pos = 70;                   // initial and then current servo position (degrees)
int pos_ms = 1200;              // initial and then current servo position (ms)
boolean servo_direction_up = 1; // keep track of which direction the servo is moving
#define MIN_PULSE  800
#define MAX_PULSE  2200

/**** OBJECTS   ****/
TMRpcm tmrpcm;   // create an object for use in this sketch
ServoTimer2 myservo;  // create servo object to control a servo 

void setup(){
  tmrpcm.speakerPin = speaker_output; //5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
  pinMode(eye_led1, OUTPUT);  // 
  pinMode(eye_led2, OUTPUT);  // 
  pinMode(sensor1, INPUT);  // 

  myservo.attach(servo1_pin);  // attaches the servo on pin 9 to the servo object 
  myservo.write(2000);
  delay(250);
  
  Serial.begin(9600);
  if (!SD.begin(SD_ChipSelectPin)) {  // see if the card is present and can be initialized:
    Serial.println("SD fail");  
    return;   // don't do anything more if not
  }
  tmrpcm.setVolume(6);
  tmrpcm.play("power-on.wav"); //the sound file "music" will play each time the arduino powers up, or is reset
  digitalWrite(eye_led1, HIGH); 
  digitalWrite(eye_led2, HIGH);
  delay(1200); 
}



void loop(){  

//    if(Serial.available()){    
//      if(Serial.read() == 'p'){ //send the letter p over the serial monitor to start playback
//        digitalWrite(eye_led1, HIGH); 
//        digitalWrite(eye_led2, HIGH); 
//        tmrpcm.play("happy2.wav");
//      }
//    }

  if(tmrpcm.isPlaying()==0){                // If sound is not playing, continue through this loop
    //tmrpcm.disable();
    digitalWrite(eye_led1, LOW); 
    digitalWrite(eye_led2, LOW);
    myservo.write(1200);
    sensor1State = digitalRead(sensor1);
    if(sensor1State == 0){                 //  If the sensor indicates blocking, then start sound/lights
      digitalWrite(eye_led1, HIGH); 
      digitalWrite(eye_led2, HIGH); 
      //myservo.write(2000);
      //delay(500);
      tmrpcm.play("happy2.wav");

    }
  }  // End of the inner loop if sound is not playing
  else {
     // Stuff that follows outside this loop will run while it is playing
 // START TESTING EYE BLINK
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis > interval) {
        // save the last time you blinked the LED 
        previousMillis = currentMillis;   
        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW)
          ledState = HIGH;
        else
          ledState = LOW;
        // set the LED with the ledState of the variable:
        digitalWrite(eye_led1, ledState);
        digitalWrite(eye_led2, ledState);
      }
  // END TESTING EYE BLINK
  
  // START TESTING HAND WAVE
      if (servo_direction_up == 1) {   // Servo direction is up for this case
        if(pos_ms >=MAX_PULSE){                 //    and it hit the upper limit
          servo_direction_up = 0;      //    reverse the direction to go down
          pos_ms = pos_ms - 1;                    //    count down
        }
        else {
          pos_ms = pos_ms + 1;                    //    Otherwise increment up
        }       
      }
      else {              /**/                   // Servo direction is down if it wasn't up before
      if(pos_ms <=MIN_PULSE) {                    //    did it hit the lower limit?
        servo_direction_up = 1;        //    reverse the direction and go up
        pos_ms = pos_ms + 1;                      //    count up
      }
      else {
        pos_ms = pos_ms - 1;                      //    Otherwise decrement down   
      }
      }
      myservo.write(pos_ms);
      delayMicroseconds(150);
  // END TESTING HAND WAVE
  }

}

// ALTERNATE SERVO SWEEP TO TRY
//      if(pos_ms <=MIN_PULSE) {                    //    did it hit the lower limit?
//        servo_direction_up = 1;        //    reverse the direction and go up
//      }
//      if(pos_ms >=MAX_PULSE){                 //    and it hit the upper limit
//          servo_direction_up = 0;      //    reverse the direction to go down
//          }
//
//      if (servo_direction_up == 1) {   // Servo direction is up for this case
//        pos_ms = pos_ms + 1;
//        }
//      if (servo_direction_up == 0) {  
//        pos_ms = pos_ms - 1;
//        }
//   delayMicroseconds(150);
//      myservo.write(pos_ms);
//

