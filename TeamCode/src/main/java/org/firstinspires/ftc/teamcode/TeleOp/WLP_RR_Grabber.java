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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file provides necessary functionality for We Love Pi Team's 2017 Relic
 * Recovery Robot's arm used for picking and stacking up 6" glyphs cube
 * into the crypto boxes.
 */


public class WLP_RR_Grabber {

    // constants
    final double spinnerPower = 0.8;
    final double stopPower = 0.0;
    final double stopPosition = 0.0;



    // Global variables to be initialized in init function
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;


    //  Declar Motors as private members
    private DcMotor spinnerLeft = null;
    private DcMotor spinnerRight = null;
    private DcMotor lift = null;

    private boolean spinIn = false;
    private boolean spinOut = false;
    private boolean pastStateA = false;
    private boolean pastStateB = false;


    private boolean isInitialized = false;

    // Use default Constructors
    WLP_RR_Grabber() {
        // Nothing needs to be done here

    }


    // Code to run ONCE when the driver hits INIT

    public void init(Telemetry telemetry, HardwareMap hardwareMap,
                     Gamepad gamepad1, Gamepad gamepad2) {


        // Initiatalize hardware devices passed from parent
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // Initialize the motors. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        spinnerLeft = hardwareMap.get(DcMotor.class, "spinnerLeft");
        spinnerRight = hardwareMap.get(DcMotor.class, "spinnerRight");
         lift = hardwareMap.get(DcMotor.class, "lift");

        isInitialized = true;
        // Send telemetry message to signify that Grabber initialization complete
        telemetry.addData("Grabber", "Initialization succeeded");
    }


    // During teleop, this function will be called repeatedly
    public void loop() {
        //double pos = armMover.getPosition();

        if (isInitialized == false) {
            return;
        }




        if (!pastStateA && gamepad2.a){
            if (spinIn) {
                spinnerLeft.setPower(stopPower);
                spinnerRight.setPower(stopPower);
                spinIn = false;

            } else {  //(if spinning out or stopped)
                spinnerLeft.setPower(spinnerPower);
                spinnerRight.setPower(-spinnerPower);
                spinOut = false;
                spinIn = true;

            }

        }
        pastStateA = gamepad2.a;


        if (!pastStateB && gamepad2.b){
            if (spinOut) {
                spinnerLeft.setPower(stopPower);
                spinnerRight.setPower(stopPower);
                spinOut = false;

            } else {  //(if spinning in or stopped)
                spinnerLeft.setPower(-spinnerPower);
                spinnerRight.setPower(spinnerPower);
                spinIn = false;
                spinOut = true;

            }

        }
        pastStateB = gamepad2.b;



        //lift: RT - up, LT - down
        if (gamepad2.left_trigger > 0.0) {
            lift.setPower(-gamepad2.left_trigger * 0.5);
        } else if (gamepad2.right_trigger > 0.0) {
            lift.setPower(gamepad2.right_trigger * 0.5);
        } else {
            if (lift.getPower() != stopPower) {
                lift.setPower(stopPower);
            }
        }

        telemetry.addData("Gamepad2 ", "left trigger (%.2f)", gamepad2.left_trigger);
        telemetry.addData("Gamepad2 ", "right trigger (%.2f)", gamepad2.right_trigger);
        telemetry.addData("Gamepad2 ", "A (%b)", gamepad2.a);
        telemetry.addData("Gamepad2 ", "B (%b)", gamepad2.b);
        telemetry.addData("Gamepad2 ", "X (%b)", gamepad2.x);
        telemetry.addData("Gamepad2 ", "Y (%b)", gamepad2.y);

        telemetry.addData("Motor ", "Spinner Left (%.2f)", spinnerLeft.getPower());
        telemetry.addData("Motor ", "Spinner Right (%.2f)", spinnerRight.getPower());
        telemetry.addData("Motor ", "lift  (%.2f)", lift.getPower());
    }
}