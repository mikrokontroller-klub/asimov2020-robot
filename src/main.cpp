#include <Arduino.h>

// PWM-able pins (Nano): 3, 5, 6, 9, 10, 11

// Pin declarations...

#define FIRST_MOTOR_SPEED_LEVEL_PWM 51
#define SECOND_MOTOR_SPEED_LEVEL_PWM 102
#define THIRD_MOTOR_SPEED_LEVEL_PWM 153
#define FOURTH_MOTOR_SPEED_LEVEL_PWM 204
#define FIFTH_MOTOR_SPEED_LEVEL_PWM 255

struct ControlState {
  struct MotorState {
    byte direction;
    byte speed;
  };

  struct ServoState {
    byte angle;
    byte speed;
  };

  MotorState leftMotor;
  MotorState rightMotor;
  ServoState servo1;
  ServoState servo2;
  ServoState servo3;
  ServoState servo4;
};

boolean newData = false;

const byte numChars = 32; // The length of the array
char receivedChars[numChars]; // The char array of the current input

void listen() {
  static boolean receivingInProgress = false;
  static byte index = 0;
  char startMarker = '#';
  char endMarker = '*';
  char currentChar;

  while (Serial.available() > 0 && newData == false) {
    currentChar = Serial.read();

    if (currentChar != endMarker) {
      receivedChars[index] = currentChar;
      index++;
      if (index >= numChars) {
        index = numChars - 1;
      }
    } else {                       // When got them all
      receivedChars[index] = '\0'; // terminate the string
      receivingInProgress = false;
      index = 0;
      newData = true;
    }
  }
}

struct ControlState parsePayloadToControlState(String input) {
  char splitDelimiter = ',';
  struct ControlState controlState = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

  int values[12], r=0, t=0;

  for (int i=0; i < input.length(); i++)
  { 
    if(input.charAt(i) == splitDelimiter) 
    { 
      values[t] = input.substring(r, i).toInt(); 
      r=(i+1); 
      t++; 
    }
  }

  controlState.rightMotor.direction = values[0];
  controlState.rightMotor.speed = values[1];
  controlState.leftMotor.direction = values[2];
  controlState.leftMotor.speed = values[3];
  controlState.servo1.angle = values[4];
  controlState.servo1.speed = values[5];
  controlState.servo2.angle = values[6];
  controlState.servo2.speed = values[7];
  controlState.servo3.angle = values[8];
  controlState.servo3.speed = values[9];
  controlState.servo4.angle = values[10];
  controlState.servo4.speed = values[11];

  return controlState;
}



void dispatchControlState(struct ControlState controlState) {
  // Handling motors...
}

void setup() {
  Serial.begin(38400);
  Serial.println("#Ready*");
}

void loop() {
  listen();
  if (newData) {
    Serial.println("#Received*");
    struct ControlState controlState = parsePayloadToControlState(receivedChars);
    dispatchControlState(controlState);
    newData = false;
  }
}