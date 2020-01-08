package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.CurvePoint;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.followCurve;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 11/6/2019
 * Example of how to use Pure Pursuit.
 */
@Autonomous(name = "Drive Wheel Pure Pursuit Test", group = "Pure Pursuit")
public class DriveWheelBasedPurePursuitTest extends OpMode {

    @Override
    public void init() {
        DriveWheelPurePursuitMovement movement = new DriveWheelPurePursuitMovement(telemetry, hardwareMap);
        movement.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 50, 0.3, 0.3, 25));
        allPoints.add(new CurvePoint(50, 50, 0.3, 0.3, 25));
        allPoints.add(new CurvePoint(50, 0, 0.3, 0.3, 25));
        allPoints.add(new CurvePoint(0, 0, 0.3, 0.3, 25));
        //TODO: MAKE THIS BETTER
        //WARNING: Robot will get stuck oscillating while looking for another point to go to at the endPoint.
        followCurve(allPoints, Math.toRadians(0));
    }
}
