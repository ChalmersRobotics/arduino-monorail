// Components: 2x servo for arms, 2x limit sensors for start position and end
// position, 1 Arduino Nano, 1x L293D, 1x magnet sensor

#include <AccelStepper.h>

enum State { INIT, HOME, RUN, POKE, CALIBRATE, PAUS };

enum Rotation { COUNTER_CLOCKWISE, CLOCKWISE };

enum PinMappings {
  BUTTON_START,
  BUTTON_CALIBRATION,
  SENSOR_MAGNET,
  SENSOR_BACK,
  SENSOR_FRONT,
  ARM_STEP_STEPPER,
  ARM_STEP_DIR,
  ARM_STEP_LIMIT_1,
  ARM_STEP_LIMIT_2,
  ENGINE_STEP_PWM,
  ENGINE_STEP_CTRL_1,
  ENGINE_STEP_CTRL_2,
  LED_ESP_CTRL,
  _SIZE_LIMIT,
};

// clang-format off
static uint8_t const pin[_SIZE_LIMIT] = {
  [BUTTON_START] = 10,
  [BUTTON_CALIBRATION] = 11,
  [SENSOR_MAGNET] = A1,
  [SENSOR_BACK] = 6,
  [SENSOR_FRONT] = 7,        
  [ARM_STEP_STEPPER] = 5,
  [ARM_STEP_DIR] = 4,
  [ARM_STEP_LIMIT_1] = 3,
  [ARM_STEP_LIMIT_2] = 2,
  [ENGINE_STEP_PWM] = A5,
  [ENGINE_STEP_CTRL_1] = 8,
  [ENGINE_STEP_CTRL_2] = 9,
  [LED_ESP_CTRL] = 12,
};
// clang-format on

// Variables go here (needs some more structure):
State state = INIT;
State nextState;
State oldState;

// Stop int for pokearm
int armstop = 0;

// To store positioning of servo arms
int pos = 0;

int stop = 0;

int motorInterfaceType = 1;

// Stepmotor push speed
int pushSpeed = 25;

int calibrationCounter = 0;

// Dc motor speed
int motorSpeed = 150;

AccelStepper stepper =
    AccelStepper(motorInterfaceType, pin[ARM_STEP_STEPPER], pin[ARM_STEP_DIR]);

void setup() {
  // Startup
  Serial.begin(9600);

  // Setup output pin modes step motor
  pinMode(pin[ARM_STEP_LIMIT_1], INPUT_PULLUP);
  pinMode(pin[ARM_STEP_LIMIT_2], INPUT_PULLUP);
  stepper.setMaxSpeed(1000); // Max speed, for korv safety
  stepper.setAcceleration(
      30); // Max accelerartion, for not unintented pushing switches

  // Setup output pin modes
  pinMode(pin[ENGINE_STEP_PWM], OUTPUT);
  pinMode(pin[ENGINE_STEP_CTRL_1], OUTPUT);
  pinMode(pin[ENGINE_STEP_CTRL_2], OUTPUT);

  // Make sure motors are off initially
  digitalWrite(pin[ENGINE_STEP_CTRL_1], LOW);
  digitalWrite(pin[ENGINE_STEP_CTRL_2], LOW);

  pinMode(pin[SENSOR_BACK], INPUT_PULLUP);
  pinMode(pin[SENSOR_FRONT], INPUT_PULLUP);
  pinMode(pin[BUTTON_START], INPUT_PULLUP);
  pinMode(pin[BUTTON_CALIBRATION], INPUT_PULLUP);
  pinMode(pin[LED_ESP_CTRL], OUTPUT);
}

void startMotor(Rotation rotation, int speed) {
  // code for starting the train, change int value to control speed
  analogWrite(pin[ENGINE_STEP_PWM], speed);
  if (rotation == CLOCKWISE) {
    digitalWrite(pin[ENGINE_STEP_CTRL_1], HIGH);
    digitalWrite(pin[ENGINE_STEP_CTRL_2], LOW);
  } else {
    digitalWrite(pin[ENGINE_STEP_CTRL_1], LOW);
    digitalWrite(pin[ENGINE_STEP_CTRL_2], HIGH);
  }
}

void stopMotor() {
  digitalWrite(pin[ENGINE_STEP_CTRL_1], LOW);
  digitalWrite(pin[ENGINE_STEP_CTRL_2], LOW);
}

void pokeAction() {
  if (stop == 0) {
    for (int i = 0; i < 2; i++) {
      int switchState = digitalRead(pin[ARM_STEP_LIMIT_1]);
      if (i == 0) {
        while (switchState == LOW) {
          stepper.setSpeed(pushSpeed);
          stepper.runSpeed();
          switchState = digitalRead(pin[ARM_STEP_LIMIT_1]);
        }
        stepper.setCurrentPosition(0);
        toggleLights();
      }
      if (i == 1) {
        switchState = digitalRead(pin[ARM_STEP_LIMIT_2]);
        int k = 0;
        while (switchState == LOW) {
          stepper.setSpeed(-pushSpeed);
          stepper.runSpeed();
          switchState = digitalRead(pin[ARM_STEP_LIMIT_2]);
        }
        toggleLights();
      }
    }

    int stopPOS = stepper.currentPosition();
    while (stepper.currentPosition() != stopPOS / 2) {
      stepper.setSpeed(25);
      stepper.runSpeed();
    }
    stop = 1;
  }
  stop = 0;
}

void toggleLights() {
    digitalWrite(pin[LED_ESP_CTRL], LOW);
    delay(100);
    digitalWrite(pin[LED_ESP_CTRL], HIGH);
    delay(100);
    digitalWrite(pin[LED_ESP_CTRL], LOW);
}

void reversePolarity() {
  if (pin[ENGINE_STEP_CTRL_1] == HIGH) {
    digitalWrite(pin[ENGINE_STEP_CTRL_1], LOW);
    digitalWrite(pin[ENGINE_STEP_CTRL_2], HIGH);
  } else {
    digitalWrite(pin[ENGINE_STEP_CTRL_1], HIGH);
    digitalWrite(pin[ENGINE_STEP_CTRL_2], LOW);
  }
}

void moveAvoidMagnetReadTwice() {
  startMotor(CLOCKWISE, motorSpeed);
  delay(1000);
}

void doStateAction() {
  switch (state) {
  case INIT: {
    // DO NOTHING
    stopMotor();
    break;
  }

  case HOME: {
    startMotor(COUNTER_CLOCKWISE, motorSpeed);
    break;
  }
  case RUN: {
    startMotor(CLOCKWISE, motorSpeed);
    break;
  }

  case POKE: {
    stopMotor();
    pokeAction();
    moveAvoidMagnetReadTwice();
    break;
  }
  case CALIBRATE: {
    stopMotor();
    if (digitalRead(pin[BUTTON_START]) == LOW) {
      pokeAction();
      calibrationCounter++;
    }
    break;
  }
  case PAUS: {
    stopMotor();
    delay(10000);
    break;
  }
  }
}

void handleStateMachine() {
  nextState = state;
  switch (state) {
  case INIT: {
    if (digitalRead(pin[BUTTON_START]) == LOW) {
      pokeAction();
      nextState = HOME;
    }
    break;
  }

  case HOME: {
    if (digitalRead(pin[SENSOR_BACK]) == LOW) {
      nextState = PAUS;
    }
    break;
  }

  case RUN: {
    if (analogRead(pin[SENSOR_MAGNET]) < 100) {
      if (digitalRead(pin[BUTTON_CALIBRATION]) == LOW) {
        nextState = CALIBRATE;
      } else {
        nextState = POKE;
      }
    }
    if (digitalRead(pin[SENSOR_FRONT]) == LOW) {
      nextState = HOME;
    }
    break;
  }
  case POKE: {
    nextState = RUN;
    break;
  }
  case CALIBRATE: {
    if (calibrationCounter >= 3) {
      moveAvoidMagnetReadTwice();
      nextState = RUN;
      calibrationCounter = 0;
    }
    break;
  }
  case PAUS: {
      nextState = RUN;
      break;
  }
  }
}

void loop() {
  //Serial.println(state); //Uncomment for testing states
  doStateAction();
  handleStateMachine();
  state = nextState;
  
}
