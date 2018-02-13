package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Swagster_Wagster on 2/10/18.
 */

@Autonomous(name = "encoders", group = "We Love Pi")
public class encoders extends LinearOpMode {

    protected DcMotor frontLeft = null;
    protected DcMotor frontRight = null;
    protected DcMotor rearLeft = null;
    protected DcMotor rearRight = null;

    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize motors.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        telemetry.addData("init", "yeet");
        telemetry.update();

        waitForStart();

        moveWithEncoder(10);

    }

    public void moveWithEncoder (int distance) {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.3);

        while(frontLeft.isBusy()) {

            telemetry.addData("current f_l " , frontLeft.getCurrentPosition());
            telemetry.update();

        }
        frontLeft.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

