/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     vexuser.c                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    7 May 2013                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  XX XXX 2013 - Initial release                         */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <stdlib.h>

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"        // vex library header
#include "robotc_glue.h"

// Digi IO configuration
static vexDigiCfg dConfig[kVexDigital_Num] = { { kVexDigital_1,
		kVexSensorDigitalInput, kVexConfigQuadEnc1, kVexQuadEncoder_1 }, {
		kVexDigital_2, kVexSensorDigitalInput, kVexConfigQuadEnc2,
		kVexQuadEncoder_1 }, { kVexDigital_3, kVexSensorDigitalInput,
		kVexConfigQuadEnc1, kVexQuadEncoder_2 }, { kVexDigital_4,
		kVexSensorDigitalInput, kVexConfigQuadEnc2, kVexQuadEncoder_2 }, {
		kVexDigital_5, kVexSensorDigitalInput, kVexConfigQuadEnc1,
		kVexQuadEncoder_3 }, { kVexDigital_6, kVexSensorDigitalOutput,
		kVexConfigQuadEnc2, kVexQuadEncoder_3 }, { kVexDigital_7,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, { kVexDigital_8,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, { kVexDigital_9,
		kVexSensorDigitalOutput, kVexConfigOutput, 0 }, { kVexDigital_10,
		kVexSensorDigitalOutput, kVexConfigOutput, 0 }, { kVexDigital_11,
		kVexSensorDigitalOutput, kVexConfigOutput, 0 }, { kVexDigital_12,
		kVexSensorDigitalOutput, kVexConfigOutput, 0 } };

//Motor Configuration
static vexMotorCfg mConfig[kVexMotorNum] = { { kVexMotor_1, kVexMotor393T,
		kVexMotorReversed, kVexSensorQuadEncoder, kVexQuadEncoder_3 }, {
		kVexMotor_2, kVexMotor393S, kVexMotorNormal, kVexSensorNone, 0 }, {
		kVexMotor_3, kVexMotor393S, kVexMotorNormal, kVexSensorQuadEncoder,
		kVexQuadEncoder_1 }, { kVexMotor_4, kVexMotor393S, kVexMotorReversed,
		kVexSensorNone, 0 }, { kVexMotor_5, kVexMotor393S, kVexMotorReversed,
		kVexSensorQuadEncoder, kVexQuadEncoder_2 }, { kVexMotor_6,
		kVexMotor393T, kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_7,
		kVexMotor393T, kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_8,
		kVexMotor393S, kVexMotorNormal, kVexSensorNone, 0 }, { kVexMotor_9,
		kVexMotor393S, kVexMotorNormal, kVexSensorNone, 0 },
		{ kVexMotor_10, kVexMotor393T, kVexMotorNormal, kVexSensorQuadEncoder,
				kVexQuadEncoder_3 } };

//Motor Port Enumerators
typedef enum {
	intakeLeft = 0,
	driveFrontLeft = 1,
	driveRearLeft = 2,
	driveFrontRight = 3,
	driveRearRight = 4,
	liftUpperLeft = 5,
	liftLowerLeft = 6,
	liftUpperRight = 7,
	liftLowerRight = 8,
	intakeRight = 9,

	MOTORNum
} MOTOR;

// Below, we created a custom timer task which constantly runs a simple timer
// It counts up from zero in 100 millisecond increments

int timer = 0;

// memory for task - use the WORKING_AREA macro
static WORKING_AREA(waVexTestThread, 512);
static msg_t vexTestThread(void *arg) {
	(void) arg;
	// register the task so competition control can kill it
	vexTaskRegister("test");
	// thread processing
	while (TRUE) {
		timer = timer + 100;
		// use vexSleep rather than chThdSleepMilliseconds
		vexSleep(100);
	}
	return (msg_t) 0;
}
void TimerFunc() {
	// Creates the test thread.
	chThdCreateStatic(waVexTestThread, sizeof(waVexTestThread), NORMALPRIO - 1,
			vexTestThread, NULL );
}

int claw = 0;

//a function to make controlling the intake motors easier-->allows for custom speed [-127,127]
void intake(int speed) {
	vexMotorSet(0, speed);
	vexMotorSet(9, speed);
}
//same as intake function but direction reversed
void outtake(int speed) {
	vexMotorSet(0, -speed);
	vexMotorSet(9, -speed);
}
//set intake motors to 0, stopping them
void stopIntake(void) {
	vexMotorSet(0, 0);
	vexMotorSet(9, 0);
}

//calculates lift velocity every 50ms averaging current velocity and last velocity
float dPositionLeftAverage, dPositionRightAverage;
int oldPositionLeft, dPositionLeft1, dPositionLeft2;
int oldPositionRight, dPositionRight1, dPositionRight2;
int liftAngleLeft, liftAngleRight;
int pL = 0, pR = 0;

void velocityStuff(void) {
	if (pL == 0) {
		//calculate change in position
		dPositionLeft1 = liftAngleLeft - oldPositionLeft;
		oldPositionLeft = liftAngleLeft;
		dPositionRight1 = liftAngleRight - oldPositionRight;
		oldPositionRight = liftAngleRight;
		//average change in positions
		dPositionLeftAverage = (dPositionLeft1 + dPositionLeft2) / 2;
		dPositionRightAverage = (dPositionRight1 + dPositionRight2) / 2;
		pL++;
	}
	//skip this this time to run at 50ms not 25ms
	else if (pL == 1)
		pL++;
	//repeat of above--> averages with last
	else if (pL == 2) {
		dPositionLeft2 = liftAngleLeft - oldPositionLeft;
		oldPositionLeft = liftAngleLeft;
		dPositionRight2 = liftAngleRight - oldPositionRight;
		oldPositionRight = liftAngleRight;
		dPositionLeftAverage = (dPositionLeft1 + dPositionLeft2) / 2;
		dPositionRightAverage = (dPositionRight1 + dPositionRight2) / 2;
		pL++;
	} else
		pL = 0;

}

/**
 * Start Autonomous Selection|
 *                           |
 *                          \ /
 * */

int isHeld = 0, button, page = 0, PAGE_MAX = 8, autonomousMode = 0, pageHolder;

/*Here we created a function that tells us whether or not the robot is enabled
 while on field control
 This will be used below for the LCD autonomous selection*/

int16_t isEnabled(void) {
	return (vexControllerCompetitonState() & kFlagDisabled ? 0 : 1);
}

//autonomous routine selection with LCD menu
//currently 3 autonomous routines
void autoSelect(void) {
	while (!isEnabled()) {
		vexLcdBacklight(0, 1);
		button = vexLcdButtonGet(0);

		vexLcdSet(0, 0, "Autonomous?");
		vexLcdSet(0, 1, "Blue  Red  Drop");

		if (button == kLcdButtonLeft)
			autonomousMode = 1;
		else if (button == kLcdButtonCenter)
			autonomousMode = 2;
		else if (button == kLcdButtonRight)
			autonomousMode = 3;

		// wait for button release
		while (vexLcdButtonGet(0) != kLcdButtonNone)
			vexSleep(25);

		//Don't hog the cpu
		vexSleep(25);
	}

}

/*
 * End Autonomous Selection / \
 *                           |
 *                           |
 */

//Variables to be used by lift controller
int commandedLiftSpeed;
int liftDifference;
float liftAdjustLeft, liftAdjustRight;
float liftSpeedLeft, liftSpeedRight;
float liftSpeedLeftUnadjusted, liftSpeedRightUnadjusted;
float liftSpeedLeftAdjusted, liftSpeedRightAdjusted;

int liftAngleLeftRaw;
int liftBiasLeft;
int shouldPID, firstPass = 1;
int liftZeroLeft, liftZeroRight;
float liftVelocityErrorLeft, liftVelocityErrorRight;
int liftTarget;
int liftErrorLeft, liftErrorRight;
void semiAutoLiftControl(void) {

	//defines right potentiometer
	liftAngleRight = vexAdcGet(0);
	//adjusts data from left potentiometer to match right pot
	liftAngleLeftRaw = 4095 - vexAdcGet(1);
	liftBiasLeft = -295 - .070 * (liftAngleLeftRaw - 900);
	liftAngleLeft = liftAngleLeftRaw + liftBiasLeft;

	//calculates average lift velocity as discussed above
	velocityStuff();

	//checks if lift buttons are pressed and sets target velocity accordingly
	if (vexControllerGet(Btn6U) == 1 && vexControllerGet(Btn8D) == 0) {
		commandedLiftSpeed = 75;
		shouldPID = 0;
	} else if (vexControllerGet(Btn6D) == 1 && vexControllerGet(Btn8D) == 0) {
		commandedLiftSpeed = -75;
		shouldPID = 0;
	} else if (vexControllerGet(Btn6U) == 1 && vexControllerGet(Btn8D) == 1) {
		commandedLiftSpeed = 25;
		shouldPID = 0;
	} else if (vexControllerGet(Btn6D) == 1 && vexControllerGet(Btn8D) == 1) {
		commandedLiftSpeed = -25;
		shouldPID = 0;
	}
	//if no button is pressed, run position controller
	else {
		commandedLiftSpeed = 0;
		shouldPID = 1;
	}

	//sets current position as target position when Cortex is first turned on
	if (firstPass == 1) {
		liftTarget = (liftAngleLeft + liftAngleRight) / 2;
		firstPass = 0;
	}

	//Velocity and Relative Position Controller
	if (shouldPID == 0) {
		//calculate differce between the heights of lift sides
		liftDifference = liftAngleLeft - liftAngleRight;
		liftAdjustLeft = -1 * liftDifference;
		liftAdjustRight = 1 * liftDifference;
		//calculate difference between current velocity and desired v
		liftVelocityErrorLeft = commandedLiftSpeed - dPositionLeftAverage;
		liftVelocityErrorRight = commandedLiftSpeed - dPositionRightAverage;
		//integrate velocity error--> get smooth acceleration
		liftSpeedLeftUnadjusted = .1 * liftVelocityErrorLeft
				+ liftSpeedLeftUnadjusted;
		liftSpeedRightUnadjusted = .1 * liftVelocityErrorRight
				+ liftSpeedRightUnadjusted;
		//cap max lift speed at +/-127
		if (liftSpeedLeftUnadjusted > 127)
			liftSpeedLeftUnadjusted = 127;
		else if (liftSpeedLeftUnadjusted < -127)
			liftSpeedLeftUnadjusted = -127;
		if (liftSpeedRightUnadjusted > 127)
			liftSpeedRightUnadjusted = 127;
		else if (liftSpeedRightUnadjusted < -127)
			liftSpeedRightUnadjusted = -127;
		//sum velocity error and difference between sides
		liftSpeedLeftAdjusted = liftSpeedLeftUnadjusted + liftAdjustLeft;
		liftSpeedRightAdjusted = liftSpeedRightUnadjusted + liftAdjustRight;
		liftSpeedLeft = liftSpeedLeftAdjusted;
		liftSpeedRight = liftSpeedRightAdjusted;

		//set target position as current position added with velocity to predict position
		liftTarget = liftAngleLeft + 2 * dPositionLeftAverage;

	}
	//position controller
	else {
		//syncronize left & right sides like above
		liftDifference = liftAngleLeft - liftAngleRight;
		liftAdjustLeft = -1 * liftDifference;
		liftAdjustRight = 1 * liftDifference;
		liftSpeedLeftUnadjusted = 0;
		liftSpeedRightUnadjusted = 0;
		//diffence between current and desired positions
		liftErrorLeft = liftTarget - liftAngleLeft;
		liftErrorRight = liftTarget - liftAngleRight;
		//adjust diffences and give them to motors
		liftSpeedLeft = .2 * liftErrorLeft + liftAdjustLeft;
		liftSpeedRight = .2 * liftErrorRight + liftAdjustRight;
	}

	//send potentiometer data to lcd
	vexLcdPrintf(0, 0, "%d  %d", liftAngleLeft, liftAngleRight);
	//vexLcdPrintf(0, 1, "%.2f  %.2f", dPositionLeftAverage,
	//		dPositionRightAverage);

	//set lift motors at above-calculated powers
	vexMotorSet(5, liftSpeedLeft);
	vexMotorSet(6, liftSpeedLeft);
	vexMotorSet(7, liftSpeedRight);
	vexMotorSet(8, liftSpeedRight);

}

//old unused intake control code--> used automatic presets, now we use manual control
int intakePreset = 0;
#define intakePosition		vexMotorPositionGet(9)
int ch5IsPressed = 0;
#define intakePreset0	0
#define intakePreset1	500
#define intakePreset2	1000
#define intakePreset3	1500
#define intakePreset4	2000
void intakePresets(void) {
	if (vexControllerGet(Btn5U) == 1 && ch5IsPressed == 0) {
		intakePreset++;
		ch5IsPressed = 1;
	} else if (vexControllerGet(Btn5D) == 1 && ch5IsPressed == 0) {
		intakePreset--;
		ch5IsPressed = 1;
	} else if (vexControllerGet(Btn5U) == 0 && vexControllerGet(Btn5D) == 0
			&& ch5IsPressed == 1)
		ch5IsPressed = 0;

	if (intakePreset == 0) {
		if (intakePosition < intakePreset0 - 50)
			intake(90);
		else if (intakePosition > intakePreset0 + 50)
			outtake(90);
		else
			intake(0);
	} else if (intakePreset == 1) {
		if (intakePosition < intakePreset1 - 50)
			intake(90);
		else if (intakePosition > intakePreset1 + 50)
			outtake(90);
		else
			stopIntake();
	} else if (intakePreset == 2) {
		if (intakePosition < intakePreset2 - 50)
			intake(90);
		else if (intakePosition > intakePreset2 + 50)
			outtake(90);
		else
			stopIntake();
	} else if (intakePreset == 3) {
		if (intakePosition < intakePreset3 - 50)
			intake(90);
		else if (intakePosition > intakePreset3 + 50)
			outtake(90);
		else
			stopIntake();
	} else if (intakePreset == 4) {
		if (intakePosition < intakePreset4 - 50)
			intake(90);
		else if (intakePosition > intakePreset4 + 50)
			outtake(90);
		else
			stopIntake();
	} else if (intakePreset > 4)
		intakePreset = 4;
	else
		intakePreset = 0;
}

//Unused fully manual intake control--> now uses auto/manual hybrid
void manualIntakeControl(void) {
	if (vexControllerGet(Btn5U) == 1 && vexControllerGet(Btn7U) == 1) {
		intake(50);
	} else if (vexControllerGet(Btn5D) == 1 && vexControllerGet(Btn7U) == 1) {
		outtake(50);
	} else if (vexControllerGet(Btn5D) == 1 && vexControllerGet(Btn7U) == 0) {
		outtake(100);
	} else if (vexControllerGet(Btn5U) == 1 && vexControllerGet(Btn7U) == 0) {
		intake(100);
	} else {
		intake(0);
	}
}

//Current Intake Control Method
void semiManualIntakeControl(void) {
	static int shouldPID;
	static int target, error;
	static int intakeCommand;
	static int derivative, previousError;

	//Direct motor control when buttons are pressed
	if (vexControllerGet(Btn5U) == 1 || vexControllerGet(Btn5D) == 1)
		shouldPID = 0;
	//use position controller to hold position when no intake buttons are pressed
	else
		shouldPID = 1;

	//set the intake motors at various velocities w/ dif buttons
	if (shouldPID == 0) {
		if (vexControllerGet(Btn5U) == 1 && vexControllerGet(Btn7U) == 1)
			intake(50);
		else if (vexControllerGet(Btn5D) == 1 && vexControllerGet(Btn7U) == 1)
			outtake(50);
		else if (vexControllerGet(Btn5D) == 1 && vexControllerGet(Btn7U) == 0)
			outtake(127);
		else if (vexControllerGet(Btn5U) == 1 && vexControllerGet(Btn7U) == 0)
			intake(127);
		target = intakePosition;
	}

	//proportional-derivative position controller for intake to hold position
	else if (shouldPID == 1) {
		//calc error using quad encoder
		error = target - intakePosition;
		//calc delta error
		derivative = error - previousError;
		previousError = error;
		//set intake motors based on P & D
		intakeCommand = 1 * error + 6 * derivative;
		intake(intakeCommand);

		//print intake data
		vexLcdPrintf(0, 1, "%d   %d", error, derivative);

	}

}

//Base control code
int driveCommandLF, driveCommandLR, driveCommandRF, driveCommandRR;
int tankCommandLF, tankCommandLR, tankCommandRF, tankCommandRR;
int strafeCommandLF, strafeCommandLR, strafeCommandRF, strafeCommandRR;

//tank-arcade hybrid
void braydonsDriveControl(void) {
	//left side forward/backward
	if (abs(vexControllerGet(Ch3)) > 5) {
		tankCommandLF = vexControllerGet(Ch3);
		tankCommandLR = vexControllerGet(Ch3);
	}
	//deadband
	else {
		tankCommandLF = 0;
		tankCommandLR = 0;
	}
	//right side forward/backward
	if (abs(vexControllerGet(Ch2)) > 5) {
		tankCommandRF = vexControllerGet(Ch2);
		tankCommandRR = vexControllerGet(Ch2);
	}
	//deadband
	else {
		tankCommandRF = 0;
		tankCommandRR = 0;
	}
	//strafing (side-to-side) for all base motors
	if (abs(vexControllerGet(Ch4)) > 5) {
		strafeCommandLF = vexControllerGet(Ch4);
		strafeCommandLR = -vexControllerGet(Ch4);
		strafeCommandRF = -vexControllerGet(Ch4);
		strafeCommandRR = vexControllerGet(Ch4);
	}
	//deadband
	else {
		strafeCommandLF = 0;
		strafeCommandLR = 0;
		strafeCommandRF = 0;
		strafeCommandRR = 0;
	}

	//set base motors w/ sum of tank and arcade-holonomic
	vexMotorSet(1, tankCommandLF + strafeCommandLF);
	vexMotorSet(2, tankCommandLR + strafeCommandLR);
	vexMotorSet(3, tankCommandRF + strafeCommandRF);
	vexMotorSet(4, tankCommandRR + strafeCommandRR);

}

//unused pneumatic claw control function--> robot no longer has a skyrise claw
int hold5 = 0;
void clawControl(void) {
	if (vexControllerGet(Btn5U) == 1 && hold5 == 0) {
		claw = 0;
		hold5 = 1;
	} else if (vexControllerGet(Btn5D) == 1 && hold5 == 0) {
		claw = 1;
		hold5 = 1;
	} else if (vexControllerGet(Btn5U) == 0 && vexControllerGet(Btn5D) == 0
			&& hold5 == 1) {
		hold5 = 0;
	}

	vexDigitalPinSet(11, claw);
}

//unused toggl between claw and conveyor--> robot no longer has claw
int toggle = 0, hold7 = 0;

void intakeClawToggle(void) {
	if (vexControllerGet(Btn7D) == 1 && hold7 == 0) {
		toggle = 1 - toggle;
		hold7 = 1;
	} else if (vexControllerGet(Btn7D) == 0 && hold7 == 1) {
		hold7 = 0;
	}
	if (toggle == 0)
		clawControl();
	else if (toggle == 1)
		manualIntakeControl();

}

//1st autonomous routine--> drops preload cube in starting tile for 1 pt
void dropCubeAuto(void) {
	//outtake
	intake(-127);
	//lift up
	vexMotorSet(5, 127);
	vexMotorSet(6, 127);
	vexMotorSet(7, 127);
	vexMotorSet(8, 127);

	//wait 1/4 second
	vexSleep(250);

	vexMotorStopAll();

	//outtake for a second
	intake(-127);

	//drive froward for a second
	vexMotorSet(1, 127);
	vexMotorSet(2, 127);
	vexMotorSet(3, 127);
	vexMotorSet(4, 127);

	//wait a second
	vexSleep(1000);

	//stop outtaking (keeps driving forward)
	intake(0);

	//wait 1/2 a second
	vexSleep(500);

	//stop all motors
	vexMotorStopAll();
	//we are done
}

//function for setting each drive motor independently in autonomous
void driveCustom(short lf, short lr, short rf, short rr) {
	vexMotorSet(1, lf);
	vexMotorSet(2, lr);
	vexMotorSet(3, rf);
	vexMotorSet(4, rr);
}
//function for setting each drive motor forward/backward @ a constant speed
void driveForward(int speed) {
	vexMotorSet(1, speed);
	vexMotorSet(2, speed);
	vexMotorSet(3, speed);
	vexMotorSet(4, speed);
}
//function for straffing @ a constant speed for all base motors
void driveStrafe(int speed) {
	vexMotorSet(1, speed);
	vexMotorSet(2, -speed);
	vexMotorSet(3, -speed);
	vexMotorSet(4, speed);
}
//function for turning in auto
void driveTurn(int speed) {
	vexMotorSet(1, speed);
	vexMotorSet(2, speed);
	vexMotorSet(3, -speed);
	vexMotorSet(4, -speed);
}

//an autonomous version of the velocity & syncro-based controller for the lift
//see above for more details
void liftControlAutoVelocity(int autoTargetVelocity) {
	liftAngleRight = vexAdcGet(0);
	liftAngleLeftRaw = 4095 - vexAdcGet(1);
	liftBiasLeft = -295 - .070 * (liftAngleLeftRaw - 900);
	liftAngleLeft = liftAngleLeftRaw + liftBiasLeft;

	velocityStuff();

	commandedLiftSpeed = autoTargetVelocity;

	liftDifference = liftAngleLeft - liftAngleRight;
	liftAdjustLeft = -1 * liftDifference;
	liftAdjustRight = 1 * liftDifference;
	liftVelocityErrorLeft = commandedLiftSpeed - dPositionLeftAverage;
	liftVelocityErrorRight = commandedLiftSpeed - dPositionRightAverage;
	liftSpeedLeftUnadjusted = .1 * liftVelocityErrorLeft
			+ liftSpeedLeftUnadjusted;
	liftSpeedRightUnadjusted = .1 * liftVelocityErrorRight
			+ liftSpeedRightUnadjusted;
	if (liftSpeedLeftUnadjusted > 127)
		liftSpeedLeftUnadjusted = 127;
	else if (liftSpeedLeftUnadjusted < -127)
		liftSpeedLeftUnadjusted = -127;
	if (liftSpeedRightUnadjusted > 127)
		liftSpeedRightUnadjusted = 127;
	else if (liftSpeedRightUnadjusted < -127)
		liftSpeedRightUnadjusted = -127;
	liftSpeedLeftAdjusted = liftSpeedLeftUnadjusted + liftAdjustLeft;
	liftSpeedRightAdjusted = liftSpeedRightUnadjusted + liftAdjustRight;
	liftSpeedLeft = liftSpeedLeftAdjusted;
	liftSpeedRight = liftSpeedRightAdjusted;

	liftTarget = liftAngleLeft + 2 * dPositionLeftAverage;

	vexLcdPrintf(0, 0, "%d  %d", liftAngleLeft, liftAngleRight);

	vexMotorSet(5, liftSpeedLeft);
	vexMotorSet(6, liftSpeedLeft);
	vexMotorSet(7, liftSpeedRight);
	vexMotorSet(8, liftSpeedRight);

}

//auto version of op control position controller for lift--> see above
void liftControlAutoPosition(int autoTargetPosition) {
	liftAngleRight = vexAdcGet(0);
	liftAngleLeftRaw = 4095 - vexAdcGet(1);
	liftBiasLeft = -295 - .070 * (liftAngleLeftRaw - 900);
	liftAngleLeft = liftAngleLeftRaw + liftBiasLeft;

	velocityStuff();

	liftTarget = autoTargetPosition;

	liftDifference = liftAngleLeft - liftAngleRight;
	liftAdjustLeft = -1 * liftDifference;
	liftAdjustRight = 1 * liftDifference;
	liftSpeedLeftUnadjusted = 0;
	liftSpeedRightUnadjusted = 0;
	liftErrorLeft = liftTarget - liftAngleLeft;
	liftErrorRight = liftTarget - liftAngleRight;
	liftSpeedLeft = .2 * liftErrorLeft + liftAdjustLeft;
	liftSpeedRight = .2 * liftErrorRight + liftAdjustRight;

	vexLcdPrintf(0, 0, "%d  %d", liftAngleLeft, liftAngleRight);

	vexMotorSet(5, liftSpeedLeft);
	vexMotorSet(6, liftSpeedLeft);
	vexMotorSet(7, liftSpeedRight);
	vexMotorSet(8, liftSpeedRight);

}

//stop all motors and wait for lcd button press--> useful for testing autonomous routines
void waitForLcd(void) {
	while (vexLcdButtonGet(0) == kLcdButtonNone) {
		vexSleep(25);
		vexMotorStopAll();
	}
}

//unused experimental autonomous to score on medium post-->unfinished
void score2CubeAutoMedPost(void) {
	//raise lift
	while (vexAdcGet(0) < 975) {
		liftControlAutoVelocity(75);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	waitForLcd();
	//intake
	while (vexMotorPositionGet(9) < 80) {
		intake(50);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	waitForLcd();
	//drive forward
	while (vexMotorPositionGet(2) < 250) {
		intake(0);
		liftControlAutoPosition(980);
		driveForward(127);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	waitForLcd();
	//intake
	while (vexMotorPositionGet(9) < 350) {
		intake(127);
		driveForward(0);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	intake(10);
	waitForLcd();
	//strafe right
	while (vexMotorPositionGet(2) > 1000) {
		driveStrafe(127);
		intake(10);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	waitForLcd();
	//raise lift to post height
	while (vexAdcGet(0) < 2300) {
		driveForward(0);
		intake(10);
		liftControlAutoVelocity(75);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	//wait for lcd
	while (vexLcdButtonGet(0) == kLcdButtonNone) {
		liftControlAutoPosition(2300);
		vexSleep(25);
	}
	//turn to face post
	while (vexMotorPositionGet(2) > -400 || vexMotorPositionGet(4) < 400) {
		if (vexMotorPositionGet(2) < 400) {
			vexMotorSet(1, 127);
			vexMotorSet(2, 127);
		} else {
			vexMotorSet(1, 0);
			vexMotorSet(2, 0);
		}
		if (vexMotorPositionGet(4) > -400) {
			vexMotorSet(3, -127);
			vexMotorSet(4, -127);
		} else {
			vexMotorSet(3, 0);
			vexMotorSet(4, 0);
		}
		liftControlAutoPosition(2300);
		intake(10);
		vexSleep(25);
	}
}

//2nd autonomous to score 2 cubes on low post--> for blue alliance
void score2CubeAutoLowPostBlue(void) {
	timer = 0;
	//raise lift
	while (vexAdcGet(0) < 975 && timer < 1000) {
		liftControlAutoVelocity(75);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//intake
	while (vexMotorPositionGet(9) < 80 && timer < 1000) {
		intake(50);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//drive forward
	while (vexMotorPositionGet(2) < 250 && timer < 3000) {
		intake(0);
		liftControlAutoPosition(980);
		driveForward(127);
		vexSleep(25);
	}
	//reset encoders
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//intake
	while (vexMotorPositionGet(9) < 350 && timer < 3000) {
		intake(127);
		driveForward(0);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//turn right
	while (vexMotorPositionGet(2) < 1000 && timer < 5000) {
		driveTurn(127);
		intake(10);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//raise lift to post height
	while (vexAdcGet(0) < 1900 && timer < 5000) {
		driveForward(0);
		intake(10);
		liftControlAutoVelocity(75);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	/*while (vexLcdButtonGet(0) == kLcdButtonNone) {
	 liftControlAutoPosition(1900);
	 vexSleep(25);
	 }*/
	//turn to face post
	while ((vexMotorPositionGet(2) < 225 || vexMotorPositionGet(4) > -225)
			&& timer < 3000) {
		liftControlAutoPosition(1900);
		if (vexMotorPositionGet(2) < 225) {
			vexMotorSet(1, 127);
			vexMotorSet(2, 127);
		} else {
			vexMotorSet(1, 0);
			vexMotorSet(2, 0);
		}
		if (vexMotorPositionGet(4) > -225) {
			vexMotorSet(3, -127);
			vexMotorSet(4, -127);
		} else {
			vexMotorSet(3, 0);
			vexMotorSet(4, 0);
		}
		intake(10);
		vexSleep(25);
	}
	//stop base and reset encoders & timer
	driveForward(0);
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	/*while (vexLcdButtonGet(0) == kLcdButtonNone) {
	 liftControlAutoPosition(1900);
	 vexSleep(25);
	 }*/
	//back up a little
	while (vexMotorPositionGet(4) > -250 && timer < 3000) {
		driveCustom(-127, -127, -60, -60);
		liftControlAutoPosition(1900);
		vexSleep(25);
	}
	vexMotorPositionSet(2,0);
	vexMotorPositionSet(4,0);
	timer=0;
	while(timer < 300){
		driveForward(-127);
		vexSleep(25);
	}
	vexMotorPositionSet(9, 0);
	driveForward(0);
	timer = 0;
	/*while (vexLcdButtonGet(0) == kLcdButtonNone) {
	 liftControlAutoPosition(1900);
	 vexSleep(25);
	 }*/
	//deposit cubes
	while (vexMotorPositionGet(9) > -1000 && timer < 3000) {
		intake(-127);
		vexSleep(25);
		liftControlAutoPosition(1900);
	}
	//backup for 1/2 a second
	driveForward(-90);
	vexSleep(500);
	vexMotorStopAll();
	//we are done

}

//3rd autonomous--> scores 2 cubes on low post--> for red alliance
void score2CubeAutoLowPostRed(void) {
	timer = 0;
	//lift up using pot or timeout
	while (vexAdcGet(0) < 975 && timer < 1000) {
		liftControlAutoVelocity(75);
		vexSleep(25);
	}
	//reset encoders and timers each time
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//intake using encoder
	while (vexMotorPositionGet(9) < 80 && timer < 1000) {
		intake(50);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//drive forward using encoder
	while (vexMotorPositionGet(2) < 250 && timer < 3000) {
		intake(0);
		liftControlAutoPosition(980);
		driveForward(127);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//intake 2nd cube using encoder
	while (vexMotorPositionGet(9) < 350 && timer < 3000) {
		intake(127);
		driveForward(0);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//turn using encoder
	while (vexMotorPositionGet(2) > -1000 && timer < 5000) {
		driveTurn(-127);
		intake(10);
		liftControlAutoPosition(980);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//waitForLcd();
	//raise arm to post height using pot
	while (vexAdcGet(0) < 2000 && timer < 5000) {
		driveForward(0);
		intake(10);
		liftControlAutoVelocity(75);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	/*while (vexLcdButtonGet(0) == kLcdButtonNone) {
	 liftControlAutoPosition(1900);
	 vexSleep(25);
	 }*/
	//turn to face post using both encoders
	while ((vexMotorPositionGet(2) > -500 || vexMotorPositionGet(4) < 500)
			&& timer < 3000) {
		liftControlAutoPosition(2000);
		if (vexMotorPositionGet(2) > -500) {
			vexMotorSet(1, -127);
			vexMotorSet(2, -127);
		} else {
			vexMotorSet(1, 0);
			vexMotorSet(2, 0);
		}
		if (vexMotorPositionGet(4) < 500) {
			vexMotorSet(3, 127);
			vexMotorSet(4, 127);
		} else {
			vexMotorSet(3, 0);
			vexMotorSet(4, 0);
		}
		intake(10);
		vexSleep(25);
	}
	driveForward(0);
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	//strafe away from the wall
	while (vexMotorPositionGet(4) < 300 && timer < 3000) {
		driveStrafe(127);
		liftControlAutoPosition(2000);
		vexSleep(25);
	}
	vexMotorPositionSet(2, 0);
	vexMotorPositionSet(4, 0);
	vexMotorPositionSet(9, 0);
	timer = 0;
	/*while (vexLcdButtonGet(0) == kLcdButtonNone) {
	 liftControlAutoPosition(1900);
	 vexSleep(25);
	 }*/
	//turn to face post
	while (vexMotorPositionGet(2) > -300 && timer < 3000) {
		driveCustom(-127, -127, 127, 127);
		liftControlAutoPosition(2000);
		vexSleep(25);
	}
	vexMotorPositionSet(9, 0);
	driveForward(0);
	timer = 0;
	/*while (vexLcdButtonGet(0) == kLcdButtonNone) {
	 liftControlAutoPosition(1900);
	 vexSleep(25);
	 }*/
	//deposit cubes on post
	while (vexMotorPositionGet(9) > -1000 && timer < 3000) {
		intake(-127);
		vexSleep(25);
		liftControlAutoPosition(2000);
	}
	//back up for 1/2 a second
	driveForward(-90);
	vexSleep(500);
	vexMotorStopAll();
	//we are done

}

/*-----------------------------------------------------------------------------*/
/** @brief      User setup                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  The digital and motor ports can (should) be configured here.
 */
void vexUserSetup() {
	vexDigitalConfigure(dConfig, DIG_CONFIG_SIZE( dConfig ));
	vexMotorConfigure(mConfig, MOT_CONFIG_SIZE( mConfig ));
}

/*-----------------------------------------------------------------------------*/
/** @brief      User initialize                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */

void vexUserInit()	// This is where all functions are initialized
{
	//start lcd autonomous selection
	autoSelect();
	//start custom timer task
	TimerFunc();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */

msg_t vexAutonomous(void *arg) {
	(void) arg;

// Must call this
	vexTaskRegister("auton");

	//run autonomous routine selected by lcd menu
	if (autonomousMode == 1)
		score2CubeAutoLowPostBlue();
	else if (autonomousMode == 2)
		score2CubeAutoLowPostRed();
	else if (autonomousMode == 3)
		dropCubeAuto();
	//or don't run a routine if none selected
	else
		vexSleep(25);

// We are done
	while (!chThdShouldTerminate()) {

		// Don't hog cpu
		vexSleep(25);
	}

	return (msg_t) 0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the driver control period is started
 */

msg_t vexOperator(void *arg) {
	(void) arg;

// Must call this
	vexTaskRegister("operator");

// Run until asked to terminate
	while (!chThdShouldTerminate()) {

		//run driver op base control
		braydonsDriveControl();

		//run operator controlled intake control
		semiManualIntakeControl();

		//run op lift controllers
		semiAutoLiftControl();

		// Don't hog cpu
		vexSleep(25);
	}

	return (msg_t) 0;
}

