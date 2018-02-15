package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Swagster_Wagster on 2/5/18.
 */

@Autonomous(name= "Auto Test Blue", group = "We Love PI")

public class TestAuto_Blue extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        myRunOpMode(.5, 15, ColorSensor.ColorName.BLUE, true ,0.5, 1800, 85,0, 0,0,-0.3,1100,0.3, 1000);


        //myRunOpMode(.5, 15, ColorSensor.ColorName.BLUE, true ,0.5, 1800, 85,0, 0,0,-0.3,1100,0.3, 1000);
    }
}
