package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.BlankedPurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 2019-12-27.
 */
@Autonomous(name = "Blanked Pure Pursuit Test", group = "PurePursuit")
public class BlankedPurePursuitTest extends LinearOpMode {

    PurePursuitController control;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialization
        control = new PurePursuitController(hardwareMap,telemetry);
        control.init();

        //Create an array to put the waypoints in.
        ArrayList<Point> waypoints = new ArrayList<>();

        //Add the waypoints into the array.
        control.addWaypoints(waypoints,
                new Point(0,0),
                new Point(10,10),
                new Point(20,10),
                new Point(30,20)
        );

        //Build the path by 1) injecting more points and then 2) smoothing it.
        control.buildPath(waypoints,6,0.01);

        waitForStart();

        telemetry.addData("CurvedPath: ", waypoints);
        telemetry.update();

        //Follow the path. The robot will point towards the last point.
        control.followPath(waypoints,0.3,0);

        while (opModeIsActive()) {
            //This is here just to read the telemetry. Pure Pursuit should usually be run using a LinearOpMode.
            //Later you should add a text map that shows a graph of your path.
        }
    }
}
