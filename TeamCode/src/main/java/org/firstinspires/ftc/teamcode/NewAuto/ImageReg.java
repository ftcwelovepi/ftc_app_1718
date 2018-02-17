package org.firstinspires.ftc.teamcode.NewAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Swagster_Wagster on 2/11/18.
 */

public class ImageReg {

    VuforiaLocalizer vuforia;

    ElapsedTime time = new ElapsedTime();

    public void init (HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQBbCRX/////AAAAGevvCPApu0phjwrtg1hDMb5FkMizE8AaJCDJtXwc8y45ZcarRtpG2Edqj3dWfUCXGTucT4Ovr/Exh6ekwjyC6D+N59nTw2BdGx9VxVquX6vBsKl2acD9cfQOBkxSs6puRSAH/Pm3FMiP4AN/LXC+VedYtfIAE2UZmIF041Lr8sLs+wgephTto8kZ8ELxQZx98Q5T/RaLo3wkz5+jYksV5Pi8VRG0vFhk47fwa0gUrZDTFhq11og/bD9zZmLiFqH3tyHeMMSkYFVakMctRPNjKYPgi9iG4jlDjGbtq2a56QxTxzjBo/J/8K2PN2A6vnBw3tMc2E1KitBtPuKrD9h/5ACsIGWh2Zo5TiQC1YGTQr6F";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    public RelicRecoveryVuMark ScanImage () {

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();

        boolean found = false;
        RelicRecoveryVuMark vuMark = null;

        while (!found && time.seconds() < 10) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT || vuMark == RelicRecoveryVuMark.CENTER) {

                found = true;
            }
        }

        return vuMark;
    }
}
