// Components: 2x servo for arms, 2x limit sensors for start position and end position, 1 Arduino Nano, 1x L293D, 1x magnet sensor

#include <Servo.h>

enum State {
  INIT, HOME, SETUP, RUN, POKE
};

enum Rotation { 
  COUNTER_CLOCKWISE, CLOCKWISE 
  };

//enum Arms = {} In case arms should have their own states

// Variables go here (needs some more structure):
State state = SETUP;
State nextState;

// Motor connections
int engine = 9;
int enginePin1 = 7;
int enginePin2 = 8;

// Start/Stop sensor
int startSensor = 5;
int stopSensor = 6;

// Magnet sensor
int magnetSensor = A1;

// Servo 1, pin 3
Servo servo1;

// Servo 2, pin 4
Servo servo2;

int startButton = 2;

// To store positioning of servo arms
int pos = 0;

void setup() {
  // Startup
  Serial.begin(9600);

  // Setup output pin modes
  pinMode(engine, OUTPUT);
  pinMode(enginePin1, OUTPUT);
  pinMode(enginePin2, OUTPUT);

  // Setup input pin modes
  pinMode(14, INPUT_PULLUP);

  // Make sure motors are off initially
  digitalWrite(enginePin1, LOW);
  digitalWrite(enginePin2, LOW);

  // Servo 1,2 on pins 3,4
  servo1.attach(3);
  servo2.attach(4);

  // Setup for sensors GPIO ports goes here:
  pinMode(startSensor, INPUT_PULLUP);
  pinMode(stopSensor, INPUT_PULLUP);

}

void startMotor(Rotation rotation, int speed) {
  // code for starting the train, change int value to control speed
  analogWrite(engine, speed);
  if (rotation == CLOCKWISE) {
    digitalWrite(enginePin1, HIGH);
    digitalWrite(enginePin2, LOW);
  }else{
    digitalWrite(enginePin1, LOW);
    digitalWrite(enginePin2, HIGH);
}
}

void stopMotor() {
  // Stops the engines
  digitalWrite(enginePin1, LOW);
  digitalWrite(enginePin2, LOW);
}

void lowerArms() {
  // TO DO: Probably better to use sensors here since 
  // delays won't be as accurate
  servo1.write(0);
  servo2.write(0);
  delay(750);
  servo1.write(90);
  servo2.write(90);
  delay(50);

}

void raiseArms() {
  // TO DO: Probably better to use sensors here since 
  // delays won't be as accurate
  servo1.write(180);
  servo2.write(180); 
  delay(750);
  servo1.write(90);
  servo2.write(90);
}

void toggle_lights() {
  // code for toggling lights on/off, currently not used. 
}

void reversePolarity() {
  if (enginePin1 == HIGH) {
    digitalWrite(enginePin1, LOW);
    digitalWrite(enginePin2, HIGH);
  } else {
    digitalWrite(enginePin1, HIGH);
    digitalWrite(enginePin2, LOW);
  }
}

void doStateAction() {
  switch (state) {
    case INIT:
      {
        // DO NOTHING
        break;
      }

    case HOME:
      {

        startMotor(COUNTER_CLOCKWISE, 255);
        break;
      }

    case SETUP:
      {
        raiseArms();
        break;
      }

    case RUN:
      {
        startMotor(CLOCKWISE, 255);
        break;
      }

    case POKE:
      {

        stopMotor();
        lowerArms();
        raiseArms();
        // To avoid re-reading the magnet sensor
        startMotor(CLOCKWISE, 255);
        delay(2000);
        break;
      }
  }
}

void handleStateMachine() {
  nextState = state;
  switch (state) {
    case INIT:
      {
        if (digitalRead(startButton) == HIGH) {
          nextState = HOME;
        }
        break;
      }

    case HOME:
      {
        if (digitalRead(startSensor) == LOW) {
          nextState = RUN;
        }
        if(digitalRead(startSensor) == LOW){
          nextState = RUN;
        }
        break;
      }

    case SETUP:
      {
        nextState = RUN;
        break;
      }

    case RUN:
      {
        if(analogRead(magnetSensor) < 100){
          Serial.println(magnetSensor);
          nextState = POKE; 
        }
        if(digitalRead(stopSensor) == LOW){
          nextState = HOME;
        }
        break;
      }

    case POKE:
      {
        nextState = RUN;
        break;
      }
  }
}

void loop() {
 // Serial.println(state); Uncomment for testing states
  doStateAction();
  handleStateMachine();
  state = nextState;
}

