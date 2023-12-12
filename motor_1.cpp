A, [29/11/2023 15.52]
#include <Stepper.h>

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"


#define steps_per_rev 200

#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25

#define SWITCH 33
bool switchState = HIGH;
bool lastSwitchState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

Stepper motor(steps_per_rev, IN1, IN2, IN3, IN4);


unsigned long startTime;
bool isExecuting = false;



MAX30105 particleSensor;

const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg; 

void setup()
{
  Serial.begin(115200);

  pinMode(SWITCH, INPUT_PULLUP); // config GPIO21 as input pin and enable the internal pull-up resistor

  motor.setSpeed(20);

  //Serial.println("Initializing...");
  Wire.begin(22,21);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop()
{
  long irValue = particleSensor.getIR();

  bool reading = digitalRead(SWITCH);

  // If the switch changed, due to noise or pressing:
  if (reading != lastSwitchState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  // Check if the switch state has stabilized
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the switch state has changed:
    if (reading != switchState) {
      switchState = reading;
      //Serial.println("---------Switch changed----------");

      // only start the 15-second window if the switch is pressed and isExecuting is false
      if (switchState == LOW && !isExecuting) {
        isExecuting = true;
        startTime = millis();
      }
    }
  }


  lastSwitchState = reading;


  if (irValue < 50000) {
    //Serial.print(" No finger?");

  } else {
    /*
    if (!isExecuting) {
      isExecuting = true;
      startTime = millis();
    }
    */
    
    if (isExecuting && millis() - startTime < 15000) {
      // Your code to run for 10 seconds goes here
      //Serial.println("Executing...");

      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable

          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      /*
      Serial.print("IR=");
      Serial.print(irValue);
      Serial.print(", BPM=");
      Serial.print(beatsPerMinute);
      Serial.print(", Avg BPM=");
      Serial.print(beatAvg);
      Serial.println();
      */
    } else if (isExecuting ) {
      // Code to execute after 10 seconds
      isExecuting = false;
      //Serial.println("15* seconds have elapsed");

A, [29/11/2023 15.52]
Serial.println(beatAvg);
      int multiplyer = 4; 
      motor.setSpeed(60);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      delay(50);
      motor.step(-beatAvg * multiplyer);
      motor.step(beatAvg * multiplyer);
      
      delay(5000);

    }

  }
    
}


//BACK AND FORTH

A, [01/12/2023 11.05]
#include <Stepper.h>

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"


#define steps_per_rev 200

#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25


#define SWITCH 33
bool switchState = HIGH;
bool lastSwitchState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

Stepper motor(steps_per_rev, IN1, IN2, IN3, IN4);


unsigned long startTime;
bool isExecuting = false;


MAX30105 particleSensor;

const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

void setup()
{
  Serial.begin(115200);

pinMode(SWITCH, INPUT_PULLUP); // config GPIO21 as input pin and enable the internal pull-up resistor

  motor.setSpeed(20);

  //Serial.println("Initializing...");
  Wire.begin(22,21);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop()
{
  long irValue = particleSensor.getIR();


  bool reading = digitalRead(SWITCH);

  // If the switch changed, due to noise or pressing:
  if (reading != lastSwitchState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  // Check if the switch state has stabilized
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the switch state has changed:
    if (reading != switchState) {
      switchState = reading;
      //Serial.println("---------Switch changed-------");

      // only start the 15-second window if the switch is pressed and isExecuting is false
      if (switchState == LOW && !isExecuting) {
        isExecuting = true;
        startTime = millis();
      }
    }
  }


  lastSwitchState = reading;

  if (irValue < 50000) {
    //Serial.print(" No finger?");

  } else {
    /*
    if (!isExecuting) {
      isExecuting = true;
      startTime = millis();
    }
    */
    if (isExecuting && millis() - startTime < 15000) {
      // Your code to run for 10 seconds goes here
      //Serial.println("Executing...");

      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable

          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      /*
      Serial.print("IR=");
      Serial.print(irValue);
      Serial.print(", BPM=");
      Serial.print(beatsPerMinute);
      Serial.print(", Avg BPM=");
      Serial.print(beatAvg);
      Serial.println();
      */
    } else if (isExecuting) {
      // Code to execute after 10 seconds
      isExecuting = false;
      //Serial.println("15* seconds have elapsed");

      Serial.print("heart||");
      Serial.println(beatAvg);

      motor.setSpeed(beatAvg/4);
      motor.step(steps_per_rev);
      //motor.setSpeed(beatAvg/4);
      motor.step(steps_per_rev);
      
      
      delay(5000);

    }

  }
    
}
