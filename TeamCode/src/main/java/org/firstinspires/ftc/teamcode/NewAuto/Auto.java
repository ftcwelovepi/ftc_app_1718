
package org.firstinspires.ftc.teamcode.NewAuto;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public abstract class Auto extends initAuto {

    public void autoOpMode(ColorSensor.ColorName teamColor, boolean runMovement, boolean right) {

        // Constants
        int sleep_time = 300;
        double grabber_speed = 0.5;
        int jewelTurn = 15;

        // init the drivetrain, colorSensor, image reg, jewel arm
        initHardware();

        telemetry.clear();

        telemetry.addData("Autonomous", "initing");
        telemetry.addData("Autonomous", "initialized");
        telemetry.update();

        // scan image
        RelicRecoveryVuMark image = imageReg.ScanImage();

        telemetry.addData("image is ", image);
        telemetry.update();

        waitForStart();

        runtime.reset();

        // afrer wait for start, code about to start
        telemetry.clear();
        telemetry.addData("Autonomous", "Auto has began");
        telemetry.addData("Autonomous", "Fasten your seat belts.");
        telemetry.update();

        // Lower the ARM
        arm.lowerArm();
        telemetry.addData("Autonomous", "Arm Lowered ");
        telemetry.update();
        sleep(1000);

        // Sense jewel
        ColorSensor.ColorName jewelColor = colorSensor.getColor();
        telemetry.clear();
        telemetry.addData("Autonomous", "JewelColor is " + jewelColor);
        telemetry.update();

        sleep(sleep_time);

        // knock jewel
        if (jewelColor != ColorSensor.ColorName.UNKNOWN) {

            if (jewelColor == teamColor) {

                jewelTurn = -jewelTurn;
            }

            telemetry.clear();
            telemetry.addData("jewel color", jewelColor);
            telemetry.addData("Autonomous", "knocking jewel");
            telemetry.update();
            drivetrain.gyroTurn(0.5, jewelTurn);

        }

        // raise arm after knocking jewel
        arm.raiseArm();
        sleep(sleep_time);


        telemetry.addData("Autonomous", "raise arm");
        telemetry.update();

        // turns back to initial position on balance board
        if (jewelColor == ColorSensor.ColorName.RED || jewelColor == ColorSensor.ColorName.BLUE) {

            jewelTurn = -jewelTurn;
            drivetrain.gyroTurn(0.5 / 1.5, jewelTurn);

        }

        // movement
        if (runMovement) {

            if (right && teamColor == ColorSensor.ColorName.BLUE) {

                sleep(sleep_time);

                int position_time = 1600;
                if (image == RelicRecoveryVuMark.LEFT) {
                    position_time = 1150;
                } else if (image == RelicRecoveryVuMark.RIGHT) {
                    position_time = 2150;
                }

                moveTime(0.5, 0, 0, position_time); // Move backwards to position.
                sleep(sleep_time);

                drivetrain.gyroTurn(0.5, 85); // Turn towards box.
                sleep(sleep_time);

                moveTime(-0.3, 0, 0, 1350); // Go towards box.
                sleep(sleep_time);

            } else if (!right && teamColor == ColorSensor.ColorName.BLUE) {

                sleep(sleep_time);

                moveTime(0.5, 0, 0, 1600); // Move off of balance.
                sleep(sleep_time);

                drivetrain.gyroTurn(0.5, -85); // Turn.
                sleep(sleep_time);


                int position_time = 300;
                if (image == RelicRecoveryVuMark.CENTER) {
                    position_time = 700;
                } else if (image == RelicRecoveryVuMark.RIGHT) {
                    position_time = 1300;
                }

                moveTime(-0.5, 0, 0, position_time); // Move forwards to position.
                sleep(sleep_time);


                drivetrain.gyroTurn(0.5, -85); // Turn towards box.
                sleep(sleep_time);

                moveTime(-0.3, 0, 0, 1550); // Go towards box.
                sleep(sleep_time);

            } else if (right && teamColor == ColorSensor.ColorName.RED) {

                sleep(sleep_time);

                moveTime(-0.5, 0, 0, 1600); // Move off of balance.
                sleep(sleep_time);

                drivetrain.gyroTurn(0.5, -85); // Turn.
                sleep(sleep_time);

                int position_time;

                if (image == RelicRecoveryVuMark.RIGHT) {
                    position_time = 300;
                } else if (image == RelicRecoveryVuMark.LEFT) {
                    position_time = 1200;
                } else {
                    position_time = 700;
                }

                moveTime(-0.5, 0, 0, position_time); // Move forwards to position.
                sleep(sleep_time);


                drivetrain.gyroTurn(0.5, 85); // Turn towards box.
                sleep(sleep_time);

                moveTime(-0.3, 0, 0, 1750); // Go towards box.
                sleep(sleep_time);

            } else if (!right && teamColor == ColorSensor.ColorName.RED) {

                sleep(sleep_time);

                int position_time = 1550;
                if (image == RelicRecoveryVuMark.CENTER) {
                    position_time = 2100;
                } else if (image == RelicRecoveryVuMark.LEFT) {
                    position_time = 2600;
                }

                moveTime(-0.5, 0, 0, position_time); // Move forwards to position.

                sleep(sleep_time);

                drivetrain.gyroTurn(0.5, 85); // Turn towards box.
                sleep(sleep_time);

                moveTime(-0.3, 0, 0, 1550); // Go towards box.
                sleep(sleep_time);
            }

            drivetrain.grabber_out(grabber_speed); // Spit out.
            sleep(sleep_time);

            moveTime(0.3, 0, 0, 1000);
        }
    }

    public void myRunOpMode(double SPEED_KNOCK, int jewelTurn, ColorSensor.ColorName teamColor, boolean runMovement, boolean subtractLeft, boolean adjustMove, double SPEED_MOVEMENT, long MOVE_TIME, int ANGLE_TURN, double CYRPTOBOX_SPEED, long CYRPTOBOX_TIME, int ANGLE_TURN_CYRPTO, double PLACE_BLOCK_SPEED, long PLACE_BLOCK_TIME, double BACKOUT_SPEED, long BACKOUT_TIME) {

        long keyAdujustment = 450; // 50 millisecond per inch at 0.3 power

        initHardware();

        telemetry.clear();

        telemetry.addData("Autonomous", "initing");
        telemetry.addData("Autonomous", "initialized");
        telemetry.update();

        RelicRecoveryVuMark image = imageReg.ScanImage();


        if (!subtractLeft) {
            keyAdujustment *= -1.0;

        }

        if (adjustMove) {

            if (image == RelicRecoveryVuMark.LEFT) {

                MOVE_TIME -= keyAdujustment;

            } else if (image == RelicRecoveryVuMark.RIGHT) {

                MOVE_TIME += keyAdujustment;
            }


        } else {

            if (image == RelicRecoveryVuMark.LEFT) {

                CYRPTOBOX_TIME -= keyAdujustment;

            } else if (image == RelicRecoveryVuMark.RIGHT) {

                CYRPTOBOX_TIME += keyAdujustment;
            }

        }


        telemetry.addData("image is ", image);
        telemetry.addData("move time is", MOVE_TIME);
        telemetry.update();

        waitForStart();

        runtime.reset();

        telemetry.clear();
        telemetry.addData("Autonomous", "Auto has began");
        telemetry.addData("Autonomous", "Fasten your seat belts.");
        telemetry.update();

        // Lower the ARM
        arm.lowerArm();
        telemetry.addData("Autonomous", "Arm Lowered ");
        telemetry.update();
        sleep(1000);

        // Knock the Jewel

        ColorSensor.ColorName jewelColor = colorSensor.getColor();
        telemetry.clear();
        telemetry.addData("Autonomous", "JewelColor is " + jewelColor);
        telemetry.update();

        mySleep(2000);

        if (jewelColor != ColorSensor.ColorName.UNKNOWN) {

            if (jewelColor == teamColor) {

                jewelTurn = -jewelTurn;
            }

            telemetry.clear();
            telemetry.addData("Autonomous", "knocking jewel");
            telemetry.update();
            drivetrain.gyroTurn(SPEED_KNOCK, jewelTurn);

        }

        arm.raiseArm();
        sleep(1000);

        telemetry.addData("Autonomous", "raise arm");
        telemetry.update();

        if (jewelColor == ColorSensor.ColorName.RED || jewelColor == ColorSensor.ColorName.BLUE) {

            jewelTurn = -jewelTurn;
            drivetrain.gyroTurn(SPEED_KNOCK / 1.5, jewelTurn);

        }

        if (runMovement == true) {

            mySleep(1000);
            moveTime(SPEED_MOVEMENT, 0, 0, MOVE_TIME);
            mySleep(1000);
            drivetrain.gyroTurn(Math.abs(SPEED_MOVEMENT), ANGLE_TURN);
            mySleep(1000);
            moveTime(CYRPTOBOX_SPEED, 0, 0, CYRPTOBOX_TIME);
            mySleep(1000);
            drivetrain.gyroTurn(Math.abs(CYRPTOBOX_SPEED * 1.25), ANGLE_TURN_CYRPTO);
            mySleep(1000);
            moveTime(PLACE_BLOCK_SPEED, 0, 0, PLACE_BLOCK_TIME);
            mySleep(1000);
            drivetrain.grabber_out(.3);
            mySleep(1000);
            moveTime(BACKOUT_SPEED, 0, 0, BACKOUT_TIME);

        }
    }

}