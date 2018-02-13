package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Swagster_Wagster on 2/6/18.
 */

public abstract  class initAuto extends LinearOpMode {


    ElapsedTime runtime = new ElapsedTime();
    JewelArm arm = new JewelArm();
    Movement drivetrain = new Movement();
    ColorSensor colorSensor = new ColorSensor();
    ImageReg imageReg = new ImageReg();

    public void initHardware() {

        arm.init(telemetry, hardwareMap);
        colorSensor.init(telemetry, hardwareMap);
        drivetrain.init(telemetry, hardwareMap, this);

    }

    public void moveTime(double x, double y, double rotation, long time) {

        drivetrain.move(x, y, rotation);
        mySleep(time);
        drivetrain.stopMotors();

    }

    public void moveWithRangeSensor (double speed, double distance) {

        if (opModeIsActive() && distance > drivetrain.rangeSensor.rawUltrasonic()) {

            while (opModeIsActive() && drivetrain.rangeSensor.rawUltrasonic() <= distance) {

                telemetry.addData("raw ultrasonic", drivetrain.rangeSensor.rawUltrasonic());
                telemetry.addData("inch", "%.2f cm", drivetrain.rangeSensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("cm", "%.2f cm", drivetrain.rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();

                drivetrain.move(-speed, 0, 0);

            }

            drivetrain.stopMotors();

        }

        if ( opModeIsActive() && distance < drivetrain.rangeSensor.rawUltrasonic()) {

            while ( opModeIsActive() && drivetrain.rangeSensor.rawUltrasonic() >= distance) {

                telemetry.addData("raw ultrasonic", drivetrain.rangeSensor.rawUltrasonic());
                telemetry.addData("inch", "%.2f cm", drivetrain.rangeSensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("cm", "%.2f cm", drivetrain.rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();

                drivetrain.move(speed, 0, 0);

            }

            drivetrain.stopMotors();

        }

    }

    public void mySleep (long time) {

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
