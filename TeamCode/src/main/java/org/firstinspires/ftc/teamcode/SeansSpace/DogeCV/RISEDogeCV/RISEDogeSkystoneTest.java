package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.RISEDogeCV;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Sean Cardosi on 2019-11-14
 */
@Autonomous(name = "Custom Doge Skystone Test", group = "DogeCV")
public class RISEDogeSkystoneTest extends LinearOpMode {

    private RISEDogeDetector.SkystonePosition skystonePosition;
    SkystoneDetector skystoneDetector;


    public void runOpMode() throws InterruptedException {
        RISEDogeDetector detector = new RISEDogeDetector(RISEDogeDetector.Cam.PHONE, hardwareMap);
        detector.skystoneDetector.useDefaults();
        detector.start();

        telemetry.addData("Stone Detected: ", detector.isDetected());

        waitForStart();

    while (opModeIsActive()) {
        telemetry.addData("Write Down x and y", " for each stone location");
        telemetry.addData("Skystone X: ", detector.getStoneX());
        telemetry.addData("Skystone Y: ", detector.getStoneY());
        telemetry.update();
        if (!detector.isDetected()) {
            telemetry.addData("SkystoneStatus", detector.isDetected());
        } else {
            skystonePosition = detector.getStoneSkystonePosition();

            if (skystonePosition == null) {
                telemetry.addData("Skystone Position: ", "UNKNOWN");
            } else if (skystonePosition == RISEDogeDetector.SkystonePosition.LEFT) {
                telemetry.addData("Skystone Position: ", "LEFT");
            } else if (skystonePosition == RISEDogeDetector.SkystonePosition.RIGHT) {
                telemetry.addData("Skystone Position: ", "RIGHT");
            } else if (skystonePosition == RISEDogeDetector.SkystonePosition.MIDDLE) {
                telemetry.addData("Skystone Position: ", "LEFT");
            }
        }
    }
    detector.stop();
    }
}