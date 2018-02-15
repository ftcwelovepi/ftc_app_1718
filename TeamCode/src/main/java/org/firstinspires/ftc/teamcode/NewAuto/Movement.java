/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.WLP_MecanumWheels;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class Movement {

    static final double     COUNTS_PER_MOTOR_REV    = 7;  // NeveRest Classic 40 Gearmotor (am-2964a)
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;   // circumference 12.57
    static final double     WHEEL_DIAMETER_CM       =  10.16;  // 4 inches = 10.16 cm

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    /* Declare OpMode members. */

    // Declare Drive Train modors
    private DcMotor spinnerLeft = null;
    private DcMotor spinnerRight = null;
    protected DcMotor frontLeft = null;
    protected DcMotor frontRight = null;
    protected DcMotor rearLeft = null;
    protected DcMotor rearRight = null;
    protected WLP_MecanumWheels wheels = new WLP_MecanumWheels();

    ModernRoboticsI2cGyro   gyro    = null; // Additional Gyro device

    ModernRoboticsI2cRangeSensor rangeSensor;


    // Global variables to be initialized in init function
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;
    private boolean isInitialized = false;
    private initAuto parent = null;


    public Movement() {
    }

    // Code to run ONCE when the driver hits INIT
    public void init(Telemetry telemetry, HardwareMap hardwareMap, initAuto parent) {


        // Initialize hardware devices passed from parent
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.parent = parent;

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("Gyro");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // Initialize motors.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        spinnerLeft = hardwareMap.get(DcMotor.class, "spinnerLeft");
        spinnerRight = hardwareMap.get(DcMotor.class, "spinnerRight");


        // Set Motor Directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        // Set autonomous mode

        //setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData("DriveByGyro", "Gyro calibration started ...");
        telemetry.update();

        ElapsedTime calTime =  new ElapsedTime();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
            parent.sleep(50);
            parent.idle();
        }

        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        gyro.resetZAxisIntegrator();

        // Send telemetry message to alert driver that we are calibrating;
        //telemetry.addData("DriveByGyro", "Gyro calibaration took " + calTime.toString());

        isInitialized = true;
        //telemetryPosition();
        //telemetry.addData("DriveByGyro", "Initialization succeeded");
    }


    public void move(double x, double y, double rotation) {

        double r = Math.hypot(x,y);
        double angle = Math.atan2(x, y) - Math.PI / 4;

        double fl = r * Math.cos(angle) - rotation;
        double fr = r * Math.cos(angle) + rotation;
        double rl = r * Math.cos(angle) - rotation;
        double rr = r * Math.cos(angle) + rotation;

        double absMax = Math.max(Math.abs(fl), Math.abs(fr));
        absMax = Math.max(Math.abs(rl), absMax);
        absMax = Math.max(Math.abs(rr), absMax);

        if (absMax > 1.0) {

            fl = fl / absMax;
            fr = fr / absMax;
            rl = rl / absMax;
            rr = rr / absMax;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
    }

    public void moveWithEncoder (double x, double y, int distance) {

        int moveAmout;

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveAmout = (int) (distance * COUNTS_PER_CM);

        frontLeft.setTargetPosition (moveAmout);
       // frontRight.setTargetPosition (moveAmout);
        rearLeft.setTargetPosition  (moveAmout);
       // rearRight.setTargetPosition (moveAmout);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

       //move(x,y, 0);

        frontLeft.setPower(x);
        rearLeft.setPower(x);

        while(frontLeft.isBusy() && rearLeft.isBusy()) {

            telemetry.addData("Front Left current ", frontLeft.getCurrentPosition());
           // telemetry.addData("Front Right current ", frontRight.getCurrentPosition());
            telemetry.addData("Rear Left current ", rearLeft.getCurrentPosition());
           // telemetry.addData("Rear Right current ", rearRight.getCurrentPosition());
            telemetry.update();

        }
        stopMotors();

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void stopMotors() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void gyroTurn(double speed, int angle) {

        gyro.resetZAxisIntegrator();
        int target = gyro.getHeading()+angle;
        if (target > 360) {
            target = target - 360;
        } else if (target < 0) {
            target = target + 360;
        }

        move(0,0, angle < 0 ? -speed : speed);

        while (parent.opModeIsActive() && (Math.abs(target - gyro.getHeading())) > HEADING_THRESHOLD) {
        }

        stopMotors();
    }

    public void oldGyroTurn ( double speed, double angle) {

        // keep looping while we are still active, and not on heading.

        // determine turn power based on +/- error
        double error = getError(angle);


        while (parent.opModeIsActive() && (Math.abs(error) > HEADING_THRESHOLD )) {

            double right_x = getSteer(error, P_TURN_COEFF);
            wheels.UpdateInput(0.0, 0.0, right_x);
            setPower(wheels);
            error = getError(angle);
            telemetry.addData("DriveByGyro::GyroTurn:error", "%.2f", error);
        }
        // Stop all motors
        setPower(0.0);
    }

    public void grabber_out (double speed) {

        //left postive
        //right negative
        spinnerLeft.setPower(speed);
        spinnerRight.setPower(-speed);

    }

    public void grabber_in (double speed) {

        //left negative
        //right power
        spinnerLeft.setPower(-speed);
        spinnerRight.setPower(speed);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    // Sets the runmode to all motoros
    private void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        rearLeft.setMode(mode);
        rearRight.setMode(mode);

        spinnerRight.setMode(mode);
        spinnerLeft.setMode(mode);
    }

    // Updates move counts to all motors
    private void adjustTargetPosition(int moveCounts) {
        frontLeft.setTargetPosition( frontLeft.getCurrentPosition() + moveCounts);
        frontRight.setTargetPosition( frontRight.getCurrentPosition() + moveCounts);
        rearLeft.setTargetPosition( rearLeft.getCurrentPosition() + moveCounts);
        rearRight.setTargetPosition( rearRight.getCurrentPosition() + moveCounts);

    }

    // Set power using mecanum wheel
    private void setPower(WLP_MecanumWheels wheels) {
        frontLeft.setPower(wheels.getFrontLeftPower());
        frontRight.setPower(wheels.getFrontRightPower());
        rearRight.setPower(wheels.getRearRightPower());
        rearLeft.setPower(wheels.getRearLeftPower());
    }

    //set specified power
    private void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        rearLeft.setPower(power);
    }


    //Update Motor position Telemetry values
    private void telemetryPosition() {

        // Display current motor positions
        telemetry.addData("Front Left Postion ", "target: %7d, current: %7d",
                frontLeft.getTargetPosition(), frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Postion ", "target: %7d, current: %7d",
                frontRight.getTargetPosition(), frontLeft.getCurrentPosition());
        telemetry.addData("Rear Left Postion ", "target: %7d, current: %7d",
                rearLeft.getTargetPosition(), rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Postion ", "target: %7d, current: %7d",
                rearRight.getTargetPosition(), rearRight.getCurrentPosition());
    }


    //Update Power Telemetry values
    private void telemetryPower() {

        // Display motor power
        telemetry.addData("frontLeft power", "%.2f", frontLeft.getPower());
        telemetry.addData("frontRight power", "%.2f", frontRight.getPower());
        telemetry.addData("rearLeft power", "%.2f", rearLeft.getPower());
        telemetry.addData("rearRight power", "%.2f", rearRight.getPower());
    }
}
