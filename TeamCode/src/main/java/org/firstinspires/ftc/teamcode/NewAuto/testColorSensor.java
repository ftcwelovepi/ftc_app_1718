package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Swagster_Wagster on 2/17/18.
 */

@Autonomous(name = "test", group = "We Love PI")
@Disabled
public class testColorSensor extends initAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        telemetry.addData("init", "yeet");
        telemetry.update();

        waitForStart();

        arm.lowerArm();

        sleep(3000);

        telemetry.clear();

        telemetry.addData("color is", colorSensor.getColor());
        telemetry.update();
        mySleep(10000);
    }
}
