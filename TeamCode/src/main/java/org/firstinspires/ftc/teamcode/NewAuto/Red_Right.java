package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Swagster_Wagster on 2/17/18.
 */

@Autonomous(name= "Red_Right", group = "We Love PI")
public class Red_Right extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        autoOpMode(ColorSensor.ColorName.RED, true, true);
    }
}

