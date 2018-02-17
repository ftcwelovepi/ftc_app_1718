package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Swagster_Wagster on 2/10/18.
 */

@Autonomous(name = "encoders", group = "We Love Pi")
@Disabled
public class encoders extends initAuto {

    @Override
    public void runOpMode() throws InterruptedException {


        initHardware();

        waitForStart();


        while (opModeIsActive()) {
            drivetrain.moveWithEncoder(0.6, 0, 1000);
        }//moveWithEncoder(.3, 0, 1000);

    }
}
