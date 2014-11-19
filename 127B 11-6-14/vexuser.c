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
// Digi IO configuration
static vexDigiCfg dConfig[kVexDigital_Num] = { { kVexDigital_1,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, { kVexDigital_2,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, { kVexDigital_3,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, { kVexDigital_4,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, { kVexDigital_5,
		kVexSensorDigitalInput, kVexConfigQuadEnc1, kVexQuadEncoder_1 }, {
		kVexDigital_6, kVexSensorDigitalInput, kVexConfigQuadEnc2,
		kVexQuadEncoder_1 }, { kVexDigital_7, kVexSensorDigitalInput,
		kVexConfigInput, 0 }, { kVexDigital_8, kVexSensorDigitalInput,
		kVexConfigInput, 0 }, { kVexDigital_9, kVexSensorDigitalOutput,
		kVexConfigOutput, 0 }, { kVexDigital_10, kVexSensorDigitalOutput,
		kVexConfigOutput, 0 }, { kVexDigital_11, kVexSensorDigitalOutput,
		kVexConfigOutput, 0 }, { kVexDigital_12, kVexSensorDigitalOutput,
		kVexConfigOutput, 0 } };

static vexMotorCfg mConfig[kVexMotorNum] = { { kVexMotor_1, kVexMotor393T,
		kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_2, kVexMotor393S,
		kVexMotorNormal, kVexSensorNone, 0 }, { kVexMotor_3, kVexMotor393S,
		kVexMotorNormal, kVexSensorNone, 0 }, { kVexMotor_4, kVexMotor393S,
		kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_5, kVexMotor393S,
		kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_6, kVexMotor393T,
		kVexMotorNormal, kVexSensorNone, 0 }, { kVexMotor_7, kVexMotor393T,
		kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_8, kVexMotor393S,
		kVexMotorReversed, kVexSensorQuadEncoder, kVexQuadEncoder_1 }, {
		kVexMotor_9, kVexMotor393S, kVexMotorNormal, kVexSensorIME,
		kImeChannel_1 }, { kVexMotor_10, kVexMotor393T, kVexMotorNormal,
		kVexSensorNone, 0 } };

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

void autoSelect(void) {
	while (!isEnabled()) {
		vexLcdBacklight(0, 1);
		button = vexLcdButtonGet(0);

		vexLcdSet(0, 0, "    Autonomous?");
		vexLcdSet(0, 1, "Yes       No");

		if (button == kLcdButtonLeft)
			autonomousMode = 1;
		else if (button == kLcdButtonRight)
			autonomousMode = 0;

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

int preset = 0, claw = 0;

int driveCommandLF, driveCommandLR, driveCommandRF, driveCommandRR;
int tankCommandLF, tankCommandLR, tankCommandRF, tankCommandRR;
int strafeCommandLF, strafeCommandLR, strafeCommandRF, strafeCommandRR;

void braydonsDriveControl(void) {
	if (abs(vexControllerGet(Ch3)) > 5) {
		tankCommandLF = vexControllerGet(Ch3);
		tankCommandLR = vexControllerGet(Ch3);
	} else {
		tankCommandLF = 0;
		tankCommandLR = 0;
	}
	if (abs(vexControllerGet(Ch2)) > 5) {
		tankCommandRF = vexControllerGet(Ch2);
		tankCommandRR = vexControllerGet(Ch2);
	} else {
		tankCommandRF = 0;
		tankCommandRR = 0;
	}
	if (abs(vexControllerGet(Ch1)) > 5) {
		strafeCommandLF = vexControllerGet(Ch1);
		strafeCommandLR = -vexControllerGet(Ch1);
		strafeCommandRF = -vexControllerGet(Ch1);
		strafeCommandRR = vexControllerGet(Ch1);
	} else {
		strafeCommandLF = 0;
		strafeCommandLR = 0;
		strafeCommandRF = 0;
		strafeCommandRR = 0;
	}

	vexMotorSet(1, tankCommandLF + strafeCommandLF);
	vexMotorSet(2, tankCommandLR + strafeCommandLR);
	vexMotorSet(3, tankCommandRF + strafeCommandRF);
	vexMotorSet(4, tankCommandRR + strafeCommandRR);

}

void dropCubeAuto(void) {

	vexMotorSet(5, 127);
	vexMotorSet(6, 127);
	vexMotorSet(7, 127);
	vexMotorSet(8, 127);

	vexMotorSet(1, 127);
	vexMotorSet(2, 127);
	vexMotorSet(3, 127);
	vexMotorSet(4, 127);

	vexSleep(1000);

	vexMotorStopAll();

	vexMotorSet(1, 127);
	vexMotorSet(2, 127);
	vexMotorSet(3, 127);
	vexMotorSet(4, 127);

	vexSleep(1000);

	vexMotorStopAll();

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
	autoSelect();
}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */

int error, target, previousError;
float speed;
float integral, derivative;
float Kp = .12, Ki = .002, Kd = .7;
float dT = 1;

msg_t vexAutonomous(void *arg) {
	(void) arg;

// Must call this
	vexTaskRegister("auton");

	if (autonomousMode == 1)
		dropCubeAuto();
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

		braydonsDriveControl();

		if (vexControllerGet(Btn6U) == 1) {
			preset = 0;
		} else if (vexControllerGet(Btn6D) == 1) {
			preset = 0;
		} else if (vexControllerGet(Btn8U) == 1) {
			preset = 1;
		} else if (vexControllerGet(Btn8D) == 1) {
			preset = -1;
		}

		if (preset == -1) {
			if (-1*vexImeGetCount(0) < 180) {
				vexMotorSet(5, 127);
				vexMotorSet(6, 127);
				vexMotorSet(7, 127);
				vexMotorSet(8, 127);
			} else if (-1*vexImeGetCount(0) > 220) {
				vexMotorSet(5, -127);
				vexMotorSet(6, -127);
				vexMotorSet(7, -127);
				vexMotorSet(8, -127);
			} else {
				vexMotorSet(5, 10);
				vexMotorSet(6, 10);
				vexMotorSet(7, 10);
				vexMotorSet(8, 10);
			}
		} else if (preset == 1) {
			vexMotorSet(5, 10);
			vexMotorSet(6, 10);
			vexMotorSet(7, 10);
			vexMotorSet(8, 10);
		} else if (preset == 0) {
			if (vexControllerGet(Btn6U) == 1) {
				vexMotorSet(5, 127);
				vexMotorSet(6, 127);
				vexMotorSet(7, 127);
				vexMotorSet(8, 127);
			} else if (vexControllerGet(Btn6D) == 1) {
				vexMotorSet(5, -127);
				vexMotorSet(6, -127);
				vexMotorSet(7, -127);
				vexMotorSet(8, -127);
			} else {
				vexMotorSet(5, 0);
				vexMotorSet(6, 0);
				vexMotorSet(7, 0);
				vexMotorSet(8, 0);
			}
		}

		if (vexControllerGet(Btn5U) == 1) {
			vexMotorSet(9, 127);
			vexMotorSet(0, 127);
		} else if (vexControllerGet(Btn5D) == 1) {
			vexMotorSet(9, -127);
			vexMotorSet(0, -127);
		} else {
			vexMotorSet(9, 0);
			vexMotorSet(0, 0);
		}

		if (vexControllerGet(Btn7U) == 1)
			claw = 1;
		else if (vexControllerGet(Btn7D) == 1)
			claw = 0;

		vexDigitalPinSet(11, claw);

		vexLcdPrintf(0,0,"%d", vexImeGetCount(0));

		// Don't hog cpu
		vexSleep(25);
	}

	return (msg_t) 0;
}

