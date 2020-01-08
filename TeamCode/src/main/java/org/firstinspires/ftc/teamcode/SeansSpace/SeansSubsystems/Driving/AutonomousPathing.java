package org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.GGOpenCV;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.GGSkystoneDetector;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.VisionSystem;

/**
 * Created by Sean Cardosi on 10/22/2019
 * Class used during autonomous initialization that reads the Skystones location and uses a predefined path
 * based off of the Skystone locations. TODO: Finish this class
 */
public class AutonomousPathing {

    private String path14 = "1or4";//Path to follow if the dice roll was a 1 or a 4
    private String path25 = "2or5";//Path to follow if the dice roll was a 2 or a 5
    private String path36 = "3or6";//Path to follow if the dice roll was a 3 or a 6
    private String unknown = "unknown";
    double pose = 0;
    GGSkystoneDetector vision;
    GGOpenCV detector;

    public AutonomousPathing(HardwareMap hardwareMap) {
        GGOpenCV detector = new GGOpenCV(GGOpenCV.Cam.PHONE, hardwareMap);
    }
    public void init() {

        detector.startCamera();
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

    }
    public void initSearch(Telemetry telemetry) {

            if (detector.found()){
                telemetry.addData("Skystone Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);

                pose = detector.detector.foundRectangle().x;

            } else {
                telemetry.addData("Skystone not found.", "");
            }

            telemetry.update();
    }
    public void stopSearch() {

        detector.stopLook();
    }

    public String findPath() {

        if ((pose>=260&&pose<=280)||(pose>=40&&pose<60)) {//If Left
            return path14;//A 1 or a 4 were rolled.

        } else if ((pose>=190&&pose<=210)||(pose>=6&&pose<=10)) {//If Middle
            return path25;//A 2 or a 5 were rolled.

        } else if ((pose>=124&&pose<=130)||(pose==0)) {//If Right
            return path36;//A 3 or a 6 were rolled.

        } else {//Could not determine a pose within a range
            return unknown;
        }
    }
}
