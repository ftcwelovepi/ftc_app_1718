package org.firstinspires.ftc.teamcode.SwagsterWagster_UltimateCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Swagster_Wagster on 1/29/18.
 */

@TeleOp(name = "testFile", group = "We Love PI")
@Disabled
public class testFile extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        Servo servo;

        servo = hardwareMap.get(Servo.class, "jewel_servo");

        telemetry.addData("init", "yeet");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.x) {

                servo.setPosition(0);

                telemetry.addData("servo is", servo.getPosition());
                telemetry.addData("servo is", "up");
                telemetry.update();


                sleep(3000);
            }
            servo.setPosition(1);

            telemetry.addData("servo is", servo.getPosition());
            telemetry.addData("servo is", "down");
            telemetry.addData("yeet fam, we out here", gamepad1.b);
            telemetry.update();

            if (gamepad1.b) {

                telemetry.addData("yeet fam, we out here", gamepad1.b);
                telemetry.update();
                sleep(3000);
            }
        }
    }
}
