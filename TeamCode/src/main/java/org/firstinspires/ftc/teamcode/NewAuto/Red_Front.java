package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Swagster_Wagster on 2/16/18.
 */

@Autonomous(name= "old_Red_Left", group = "We Love PI")
public class Red_Front  extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        myRunOpMode(.5, 15, ColorSensor.ColorName.RED, true, false,true, -0.5, 1800, 85,0, 0,0,-0.3,1100,0.3, 1000);

    }
}
