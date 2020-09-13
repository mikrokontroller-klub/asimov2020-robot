/*
 * Created by Oliver Emanuel Nagy
 *
 * GitHub repository: https://github.com/mikrokontroller-klub/asimov2020-robot
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

#define MAX_STEPPER_SPEED 8000

#define LEFT_MOTOR_DIR_PIN 3
#define LEFT_MOTOR_STEP_PIN 2
#define RIGHT_MOTOR_DIR_PIN 5
#define RIGHT_MOTOR_STEP_PIN 4

struct StepperControlState
{
	byte direction;
	byte speed;
};
struct ServoControlState
{
	byte angle;
	byte speed;
};
struct ControlState
{
	StepperControlState leftStepper;
	StepperControlState rightStepper;
	ServoControlState servo1;
	ServoControlState servo2;
	ServoControlState servo3;
	ServoControlState servo4;
};

boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];
AccelStepper leftStepper(AccelStepper::DRIVER, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_STEP_PIN);
AccelStepper rightStepper(AccelStepper::DRIVER, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_STEP_PIN);
Adafruit_PWMServoDriver pwmServoDriver = Adafruit_PWMServoDriver();

struct ControlState deserialise(String payload)
{
	char splitDelimiter = ',';
	struct ControlState controlState;

	byte values[12], r = 0, t = 0;

	for (byte i = 0; i < payload.length(); i++)
	{
		if (payload.charAt(i) == splitDelimiter)
		{
			values[t] = payload.substring(r, i).toInt();
			r = (i + 1);
			t++;
		}
	}

	controlState.rightStepper.speed = values[1];
	controlState.rightStepper.direction = values[0];
	controlState.leftStepper.direction = values[2];
	controlState.leftStepper.speed = values[3];
	controlState.servo1.speed = values[5];
	controlState.servo2.angle = values[6];
	controlState.servo2.speed = values[7];
	controlState.servo1.angle = values[4];
	controlState.servo3.angle = values[8];
	controlState.servo3.speed = values[9];
	controlState.servo4.angle = values[10];
	controlState.servo4.speed = values[11];

	return controlState;
}
int angleToPWM(byte angle)
{
	int pulse_wide, analog_value;
	pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
	return analog_value;
}
void listen()
{
	static boolean receivingInProgress = false;
	static byte index = 0;
	char openingMarker = '#';
	char closingMarker = '*';
	char currentChar;

	while (Serial.available() > 0 && newData == false)
	{
		currentChar = Serial.read();

		if (receivingInProgress == true)
		{
			if (currentChar != closingMarker)
			{
				receivedChars[index] = currentChar;
				index++;
				if (index >= numChars)
				{
					index = numChars - 1;
				}
			}
			else
			{								 // When got them all
				receivedChars[index] = '\0'; // terminate the string
				receivingInProgress = false;
				index = 0;
				newData = true;
			}
		}
		else if (currentChar == openingMarker)
		{
			receivingInProgress = true;
		}
	}
}
void dispatchStepperControlState(AccelStepper &stepper, struct StepperControlState stepperControlState)
{
	int translatedSpeed = map(stepperControlState.speed, 0, 5, 0, MAX_STEPPER_SPEED);
	switch (stepperControlState.direction)
	{
	case 0:
		stepper.setSpeed(-translatedSpeed);
		break;
	case 1:
		stepper.setSpeed(translatedSpeed);
		break;
	}
}
void dispatchServoControlState(byte servoIndex, struct ServoControlState servoControlState)
{
	pwmServoDriver.setPWM(servoIndex, 0, angleToPWM(servoControlState.angle));
}
void handleControlStateChange(struct ControlState newControlState)
{
	dispatchStepperControlState(leftStepper, newControlState.leftStepper);
	dispatchStepperControlState(rightStepper, newControlState.rightStepper);
	dispatchServoControlState(0, newControlState.servo1);
	dispatchServoControlState(1, newControlState.servo2);
	dispatchServoControlState(2, newControlState.servo3);
	dispatchServoControlState(3, newControlState.servo4);
}

void setup()
{
	Serial.begin(9600);
	pwmServoDriver.begin();
	pwmServoDriver.setPWMFreq(FREQUENCY);
	leftStepper.setMaxSpeed(MAX_STEPPER_SPEED);
	rightStepper.setMaxSpeed(MAX_STEPPER_SPEED);
	Serial.println("#Ready*");
}

void loop()
{
	listen();
	if (newData)
	{
		Serial.println("#Received*");
		struct ControlState controlState = deserialise(receivedChars);
		handleControlStateChange(controlState);
		newData = false;
	}

	leftStepper.runSpeed();
	rightStepper.runSpeed();
}