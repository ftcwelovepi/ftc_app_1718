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


/**
 * This file provides necessary functionality for We Love Pi Team's 2017 Relic
 * Recovery Robot's color sensor functionality to sense jewel color so that we
 * can knock the jewel belong to the opposing team and leaving our color jewel
 * intact on its origial position.
 */

public class ColorSensor {


    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     * <p>
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     * <p>
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     * <p>
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     */

    // Constants
    public enum ColorName {
        RED, BLUE, GREEN, UNKNOWN
    }

    static public final int COLOR_THRESHOLD = 6;
    // Global variables to be initialized in init function
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;
    private com.qualcomm.robotcore.hardware.ColorSensor sensorColor = null;
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

        // Detect 10 times and get the average
        int tryCount = 10;

        int red = 0;
        int blue = 0;
        int green = 0;

        for (int i=0; i<tryCount; i++) {
            red += sensorColor.red();
            blue += sensorColor.blue();
            green += sensorColor.green();
            sleep(20);
        }

        red = red/tryCount;
        blue = blue/tryCount;

        if (red >= COLOR_THRESHOLD && red > blue) {
            return ColorName.RED;
        }

        if (blue >= COLOR_THRESHOLD && blue > red) {
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
