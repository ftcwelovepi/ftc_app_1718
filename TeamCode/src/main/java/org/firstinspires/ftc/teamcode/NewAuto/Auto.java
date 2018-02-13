/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.NewAuto;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This is the We Love Pi Relic Recovery autonomous mode main entry point. The We Love Pi autonomous
 * mode includes a Jewel Arm and a Mecanum Wheel Drivetrain steered with a Modern Robotics gyro sensor.
 */

public abstract class Auto extends initAuto {

    public void myRunOpMode(double SPEED_KNOCK, int jewelTurn, ColorSensor.ColorName teamColor, boolean runMovement, double SPEED_MOVEMENT, long MOVE_TIME, double DISTANCE_MOVE, int ANGLE_TURN, double FINAL_SPEED, double FINAL_DISTANCE) {

        initHardware();

        telemetry.clear();

        telemetry.addData("Autonomous", "initing");
        telemetry.addData("Autonomous", "initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        telemetry.clear();
        telemetry.addData("Autonomous", "Auto has began");
        telemetry.addData("Autonomous", "Fasten your seat belts.");
        telemetry.update();

        // Lower the ARM
        arm.lowerArm();
        telemetry.addData("Autonomous", "Arm Lowered ");
        telemetry.update();
        sleep(1000);

        // Knock the Jewel

        ColorSensor.ColorName jewelColor = colorSensor.getColor();
        telemetry.clear();
        telemetry.addData("Autonomous", "JewelColor is " + jewelColor);
        telemetry.update();

        if (jewelColor != ColorSensor.ColorName.UNKNOWN) {

            if (jewelColor == teamColor) {

                jewelTurn = -jewelTurn;
            }

            telemetry.clear();
            telemetry.addData("Autonomous", "knocking jewel");
            telemetry.update();
            drivetrain.gyroTurn(SPEED_KNOCK, jewelTurn);

        }

        arm.raiseArm();
        sleep(1000);

        telemetry.addData("Autonomous", "raise arm");
        telemetry.update();

        if (jewelColor == ColorSensor.ColorName.RED || jewelColor == ColorSensor.ColorName.BLUE) {

            jewelTurn = -jewelTurn;
            drivetrain.gyroTurn(SPEED_KNOCK / 1.5, jewelTurn);

        }

        if (runMovement == true) {

            mySleep(1000);
            moveTime(SPEED_MOVEMENT, 0, 0, MOVE_TIME);
            mySleep(1000);
            drivetrain.gyroTurn(Math.abs(SPEED_MOVEMENT), ANGLE_TURN);
            mySleep(1000);
            moveTime(-0.3, 0, 0, 1200);
            drivetrain.grabber_out(.3);
            mySleep(1000);
            moveTime(.3, 0,0, 1000);

        }


    }
}