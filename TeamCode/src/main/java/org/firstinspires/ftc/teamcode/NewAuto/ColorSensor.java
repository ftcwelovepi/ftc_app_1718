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

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {


    // Constants
    public enum ColorName {
        RED, BLUE, GREEN, UNKNOWN
    }

    static public final int COLOR_THRESHOLD = 12;
    // Global variables to be initialized in init function
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;
    public com.qualcomm.robotcore.hardware.ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;
    private boolean isInitialized = false;
    // Default constructor
    public ColorSensor() {

    }

    // Code to run ONCE when the driver hits INIT
    public void init(Telemetry telemetry, HardwareMap hardwareMap) {


        // Initialize hardware devices passed from parent
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;


        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "color_sensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");

        isInitialized = true;
        telemetry.addData("Color Sensor", "Initialization succeeded");
    }

    public void testColor() {

        int r = sensorColor.red();
        int g = sensorColor.green();
        int b = sensorColor.blue();
        int a = sensorColor.alpha();

        telemetry.addData("R:", r);
        telemetry.addData("G:", g);
        telemetry.addData("B:", b);
        telemetry.addData("A:", a);
        telemetry.addData("jewel is ", getColor());

        telemetry.update();
    }

    // Main method that returns currently seen color name
    public ColorName getColor() {

        int red = 0;
        int blue = 0;

        for (int i = 0; i < 10; i++) {


            red += sensorColor.red();
            blue += sensorColor.blue();

            telemetry.clear();
            telemetry.addData("red value ", red);
            telemetry.addData("blue value", blue);
            telemetry.update();

            sleep(100);

        }

        telemetry.clear();
        telemetry.addData("red is", red);
        telemetry.addData("blue is", blue);
        telemetry.update();
        sleep(1000);

        red = red/10;
        blue = blue /10;

        if (red - blue > COLOR_THRESHOLD && red > blue) {

            return ColorName.RED;
        }

        if (blue > COLOR_THRESHOLD && blue > red) {

            return ColorName.BLUE;
        }

        return ColorName.UNKNOWN;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
