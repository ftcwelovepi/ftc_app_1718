package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Swagster_Wagster on 2/8/18.
 */

@Autonomous(name = "Jewel_Side_Red", group = "We Love Pi")
public class Jewel_Side_Red extends Auto{

    @Override
    public void runOpMode() throws InterruptedException {

        autoOpMode(ColorSensor.ColorName.RED, false, false);

    }
}
