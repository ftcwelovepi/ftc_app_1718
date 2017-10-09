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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Holonomic bot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  Front Drive motor:        "left_drive"
 * Motor channel:  Right Front Drive motor:        "right_drive"
 * Motor channel:  Left Back Drive motor:          "left_back"
 * Motor channel:  Right Back Drive motor:         "right_back"
 */

    /*
    * Created by Roma Bhatia (@sugarcrystals01) on 9/21/17
    * Last edit: 10/7/17
    */
public class HolonomicHardware
{
    /* Public OpMode members. */
    public DcMotor  F_L = null;
    public DcMotor  F_R = null;
    public DcMotor  R_L = null;
    public DcMotor  R_R = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HolonomicHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        F_L = hwMap.get(DcMotor.class, "F_L");
        F_R = hwMap.get(DcMotor.class, "F_R");
        R_L = hwMap.get(DcMotor.class, "R_L");
        R_R = hwMap.get(DcMotor.class, "R_R");
        /*For holonomic drive: change all direction of motors to the same direction.
         * Because we are not negating/changing directions in our code (see HolonomicTeleop.java) we are
         * using REVERSE direction. */
        F_L.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors FOR TANK DRIVE
        F_R.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors FOR TANK DRIVE
        R_L.setDirection(DcMotor.Direction.REVERSE);
        R_R.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        F_L.setPower(0);
        F_R.setPower(0);
        R_L.setPower(0);
        R_R.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        F_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        F_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }
 }

