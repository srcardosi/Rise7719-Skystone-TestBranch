package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="GG Detector Test")
public class GGSkystoneDetectorTest extends LinearOpMode {

    GGSkystoneDetector vis;
    GGOpenCV detector;

    @Override
    public void runOpMode(){

        GGOpenCV detector = new GGOpenCV(GGOpenCV.Cam.PHONE, hardwareMap);
        detector.startCamera();
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        waitForStart();

        while (opModeIsActive()){

            if (detector.found()){
                telemetry.addData("SS Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);
            } else {
                telemetry.addData("SS not found.", "");
            }

            telemetry.update();

        }



    }

}
