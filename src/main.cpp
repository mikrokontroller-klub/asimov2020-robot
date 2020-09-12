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

struct StepperState
{
	byte direction;
	byte speed;
};
struct ServoState
{
	byte angle;
	byte speed;
};
struct State
{
	StepperState leftStepper;
	StepperState rightStepper;
	ServoState servo1;
	ServoState servo2;
	ServoState servo3;
	ServoState servo4;
};

boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];
AccelStepper leftStepper(AccelStepper::DRIVER, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_STEP_PIN);
AccelStepper rightStepper(AccelStepper::DRIVER, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_STEP_PIN);
Adafruit_PWMServoDriver pwmServoDriver = Adafruit_PWMServoDriver();

struct State parseToState(String payload)
{
	char splitDelimiter = ',';
	struct State state;

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

	state.rightStepper.speed = values[1];
	state.rightStepper.direction = values[0];
	state.leftStepper.direction = values[2];
	state.leftStepper.speed = values[3];
	state.servo1.angle = values[4];
	state.servo1.speed = values[5];
	state.servo2.angle = values[6];
	state.servo2.speed = values[7];
	state.servo3.angle = values[8];
	state.servo3.speed = values[9];
	state.servo4.angle = values[10];
	state.servo4.speed = values[11];

	return state;
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
	char startMarker = '#';
	char endMarker = '*';
	char currentChar;

	while (Serial.available() > 0 && newData == false)
	{
		currentChar = Serial.read();

		if (receivingInProgress == true)
		{
			if (currentChar != endMarker)
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
		else if (currentChar == startMarker)
		{
			receivingInProgress = true;
		}
	}
}
void dispatchStepperState(AccelStepper &stepper, struct StepperState stepperState)
{
	int translatedSpeed = map(stepperState.speed, 0, 5, 0, MAX_STEPPER_SPEED);
	switch (stepperState.direction)
	{
	case 0:
		stepper.setSpeed(-translatedSpeed);
		break;
	case 1:
		stepper.setSpeed(translatedSpeed);
		break;
	}
}
void dispatchServoState(byte servoIndex, struct ServoState servoState)
{
	pwmServoDriver.setPWM(servoIndex, 0, angleToPWM(servoState.angle));
}
void handleStateChange(struct State newState)
{
	dispatchStepperState(leftStepper, newState.leftStepper);
	dispatchStepperState(rightStepper, newState.rightStepper);
	dispatchServoState(0, newState.servo1);
	dispatchServoState(1, newState.servo2);
	dispatchServoState(2, newState.servo3);
	dispatchServoState(3, newState.servo4);
}

void setup()
{
	Serial.begin(38400);
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
		struct State state = parseToState(receivedChars);
		handleStateChange(state);
		newData = false;
	}

	leftStepper.runSpeed();
	rightStepper.runSpeed();
}