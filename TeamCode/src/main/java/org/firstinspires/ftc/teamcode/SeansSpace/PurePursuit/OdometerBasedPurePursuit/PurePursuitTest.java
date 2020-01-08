package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.CurvePoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.followCurve;

/**
 * Created by Sean Cardosi.
 * Example of how to use Pure Pursuit to create an autonomous.
 */
@Disabled
@Autonomous(name = "Pure Pursuit Test", group = "Pure Pursuit")
public class PurePursuitTest extends OpMode {

    @Override
    public void init() {
        PurePursuitMovement movement = new PurePursuitMovement(telemetry, hardwareMap);
        movement.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,50,1.0,1.0,25));
        allPoints.add(new CurvePoint(50,50,1.0,1.0,25));
        allPoints.add(new CurvePoint(50,0,1.0,1.0,25));
        allPoints.add(new CurvePoint(0,0,1.0,1.0,25));

        followCurve(allPoints, Math.toRadians(90));//Robot will get stuck spinning while looking for another point to go to at the endPoint.
    }
}
