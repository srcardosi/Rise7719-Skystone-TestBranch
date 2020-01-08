package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.CurvePoint;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.PurePursuitMath;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.PurePursuitMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry.PurePursuitMath.lineCircleIntersection;

/**
 * Created by Sean Cardosi.
 * PurePursuitMovement is a class containing all of the functions to find and
 * provide movement to the drivetrain while following a set of given points.
 */
public class PurePursuitMovement {

    private static Telemetry telemetry;

    static double movementX = 0.0;
    static double movementY = 0.0;
    static double movementTurn = 0.0;
    private static int lastClosestPoint = 0;
    private static int nextPossiblePoint = 0;
    private static int previousPoint = -1;

    private static PurePursuitDrivetrain pwr;
    private static Odometry odometry;

    PurePursuitMovement(Telemetry tel, HardwareMap hardwareMap) {
        telemetry = tel;

        pwr = new PurePursuitDrivetrain(hardwareMap);
        odometry = new Odometry(hardwareMap, tel);
    }


    public void init(HardwareMap hardwareMap, Telemetry tel) {
        pwr = new PurePursuitDrivetrain(hardwareMap);
        pwr.init();
        odometry = new Odometry(hardwareMap, tel);
        odometry.init();
        movementX = 0.0;
        movementY = 0.0;
        movementTurn = 0.0;
        lastClosestPoint = 0;
        nextPossiblePoint = 0;
        previousPoint = -1;
    }

    /**
     * Follows a set of given waypoints using a curving algorithm.
     * @param allPoints The array of waypoints to follow.
     * @param followAngle The robot angle to follow the path in.
     */
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(odometry.xLocation,odometry.yLocation), allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

        telemetry.addData("FollowMePoint", "(%f,%f)", followMe.x, followMe.y);
    }


    /**
     * Finds the waypoint closest to the robot.
     * @param points An array of points to test.
     * @param robotPose The robot's current location.
     * @return Returns the closest point.
     */
    private static int getClosestPointIndex(ArrayList<CurvePoint> points, Point robotPose) {
        /*
         * Currently if the point1 is closer than point0, then the robot will go to point1.
         * This only happens for point0-1.
         * TODO: This needs a fix.
         */
        double shortestDistance  = Double.MAX_VALUE;
        int closestPoint = 0;
        nextPossiblePoint = lastClosestPoint+1;
        previousPoint = lastClosestPoint-1;
        for (int i=previousPoint; i<nextPossiblePoint && i<points.size()-1; i++) {
            if (Math.hypot(points.get(i+1).x*odometry.COUNTS_PER_INCH-robotPose.x, points.get(i+1).y*odometry.COUNTS_PER_INCH-robotPose.y) < shortestDistance) {
                closestPoint = i+1;
                shortestDistance = Math.hypot(points.get(i+1).x*odometry.COUNTS_PER_INCH-robotPose.x, points.get(i+1).y*odometry.COUNTS_PER_INCH-robotPose.y);
            }
        }
        lastClosestPoint = closestPoint;
        return closestPoint;
    }

    /**
     * Gets the path for the robot to follow based on path intersections
     * @param pathPoints This should be and array of the path points
     * @param robotLocation This should be the robots location
     * @param followRadius This should be the follow distance of the path
     * @return Returns a point to follow
     * Note: Inputs should be in encoder counts. The array of points is in inches.
     */
    private static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {

        int closestPointIndex = getClosestPointIndex(pathPoints,robotLocation);

        CurvePoint followMe = new CurvePoint(pathPoints.get(closestPointIndex));
        telemetry.addData("closestPointIndex", closestPointIndex);

        for (int i=closestPointIndex; i<nextPossiblePoint && i<pathPoints.size()-1; i++) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint(), telemetry);

            telemetry.addData("Intersections", intersections);
            double closestAngle = 10000000;

            telemetry.addData("startLine",startLine.toPoint());
            telemetry.addData("endLine",endLine.toPoint());
            for (Point thisIntersection : intersections) {

                double angle = Math.atan2(thisIntersection.y - odometry.yLocation/odometry.COUNTS_PER_INCH, thisIntersection.x - odometry.xLocation/odometry.COUNTS_PER_INCH);
                double deltaAngle = Math.abs(PurePursuitMath.AngleWrap(angle - odometry.getRawHeading()));

//                telemetry.addData("IntersectionAngle", Math.toDegrees(angle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }


    /**
     * Basic run to a position. Better to use followCurve as it implements this and uses it better.
     * Note: Inputs should be in inches.
     * @param x The x location to run to
     * @param y The y location to run to
     * @param movementSpeed The speed scaling factor to run forwards and backwards movement
     * @param preferredAngle The robot angle to follow the path with in radians
     * @param turnSpeed The speed scaling factor for turning movement
     */
    private static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x*odometry.COUNTS_PER_INCH - odometry.xLocation, y*odometry.COUNTS_PER_INCH - odometry.yLocation);

//        telemetry.addData("DistanceToTarget", distanceToTarget / odometry.COUNTS_PER_INCH);

        double absoluteAngleToTarget = Math.atan2(y*odometry.COUNTS_PER_INCH - odometry.yLocation, x*odometry.COUNTS_PER_INCH - odometry.xLocation);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (odometry.getRawHeading()));

//        telemetry.addData("RelativeAngleToPoint", Math.toDegrees(relativeAngleToPoint));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        telemetry.addData("x,y to point", "%f,%f", relativeXToPoint/odometry.COUNTS_PER_INCH, relativeYToPoint/odometry.COUNTS_PER_INCH);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementX = movementXPower * movementSpeed;
        movementY = movementYPower * movementSpeed;

        pwr.ApplyPower();
        odometry.updateLocation();

        double relativeTurnAngle = -(relativeAngleToPoint + preferredAngle);
        movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1,1) * turnSpeed;

        if (distanceToTarget < 4*odometry.COUNTS_PER_INCH) {// < 4 inches
            movementTurn = 0;
        }
    }
}
