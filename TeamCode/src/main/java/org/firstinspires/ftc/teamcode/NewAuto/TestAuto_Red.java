package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Swagster_Wagster on 2/5/18.
 */

@Autonomous(name= "Auto Test Red", group = "We Love PI")

public class TestAuto_Red extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        myRunOpMode(.5, 15, ColorSensor.ColorName.RED, true , -0.5, 1300, -85,-0.3,1350,85,-0.3,1100,0.3,1000);

    }
}
