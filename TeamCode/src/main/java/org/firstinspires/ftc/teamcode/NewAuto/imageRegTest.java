package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

/**
 * Created by Swagster_Wagster on 2/11/18.
 */

@Autonomous(name = "imageReg", group = "We Love Pi")
public class imageRegTest extends LinearOpMode {

    ImageReg imageReg = new ImageReg();

    @Override
    public void runOpMode() throws InterruptedException {

        imageReg.init(hardwareMap);

        waitForStart();

        telemetry.addData("Vumark is", imageReg.ScanImage());
        telemetry.update();
        sleep(10000);
    }
}