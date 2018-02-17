package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Swagster_Wagster on 2/5/18.
 */

@Autonomous(name= "old_Blue_Right", group = "We Love PI")
public class Blue_Front extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {

        myRunOpMode(.5, 15, ColorSensor.ColorName.BLUE, true , true, true, 0.5, 1700, 85,0, 0,0,-0.3,1000,0.3, 1000);

    }
}
