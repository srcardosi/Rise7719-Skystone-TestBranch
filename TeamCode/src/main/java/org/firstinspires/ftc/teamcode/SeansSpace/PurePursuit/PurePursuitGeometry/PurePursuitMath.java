package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi.
 * PurePursuitMath is a class containing all of the math functions for use with Pure Pursuit.
 */
public class PurePursuitMath {

    private static double COUNTS_PER_REV = 537.6;
    private static double EXTERNAL_GEAR_RATIO = 0.78125;     // This is < 1.0 if geared UP
    private static double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    private static double COUNTS_PER_INCH = ((COUNTS_PER_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));

    public PurePursuitMath() {}
    /**
     * Makes sure the angle is within the range -180 to 180 degrees.
     * @param angle The angle to normalize in radians
     * @return Return the normalized angle
     */
    public static double AngleWrap(double angle) {

        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Finds all the points of intersection of the robot's extended circle and the path.
     * @param circleCenter The robot's location.
     * @param radius The follow distance in inches.
     * @param linePoint1 The first point on the line segment.
     * @param linePoint2 The second point on the line segment.
     * @param telemetry Display helpful data.
     * @return Returns all the intersections.
     */
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2, Telemetry telemetry) {

        circleCenter.x /= COUNTS_PER_INCH;
        circleCenter.y /= COUNTS_PER_INCH;

        if (linePoint1.y == linePoint2.y) {
            linePoint1.y += 0.001;
        }
        if (linePoint1.x == linePoint2.x) {
            linePoint1.x += 0.001;
        }

        double x0 = linePoint1.x;
        double y0 = linePoint1.y;
        double x_1 = linePoint2.x;
        double y_1 = linePoint2.y;

        double A = Math.pow(x_1-x0,2) + Math.pow(y_1-y0,2);
        double B = 2 * (x_1-x0) * (x0-circleCenter.x) + 2 * (y_1-y0) * (y0-circleCenter.y);
        double C = Math.pow(x0-circleCenter.x,2) + Math.pow(y0-circleCenter.y,2) - Math.pow(radius,2);
        double discriminant = Math.pow(B,2) - 4*A*C;
        if (A == 0) {//No infinities
            A += 0.001;
        }
        if (-B + Math.sqrt(discriminant) == 0 || -B - Math.sqrt(discriminant) == 0) {
            B +=0.001;
        }

        if (discriminant < 0) {
            telemetry.addData("There are","no intersections");
        } else if (discriminant >= 0) {
            telemetry.addData("There are","some intersections");
        }
        double t1 = (-B + Math.sqrt(discriminant)) / (2 * A);
        double t2 = (-B - Math.sqrt(discriminant)) / (2 * A);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {

            double x1Root = (x_1-x0) * t1 + x0;
            double y1Root = (y_1-y0) * t1 + y0;
            double x2Root = (x_1-x0) * t2 + x0;
            double y2Root = (y_1-y0) * t2 + y0;

            telemetry.addData("x1,y1","%f,%f",x1Root,y1Root);
            telemetry.addData("x2,y2","%f,%f",x2Root,y2Root);

            double minX = Math.min(linePoint1.x, linePoint2.x);
            double maxX = Math.max(linePoint1.x, linePoint2.x);
            double minY = Math.min(linePoint1.y, linePoint2.y);
            double maxY = Math.max(linePoint1.y, linePoint2.y);

            if (t1 >= 0 && t1 <= 1) {
                if (x1Root > minX && x1Root < maxX && y1Root > minY && y1Root < maxY) {
                    allPoints.add(new Point(x1Root, y1Root));
                }
            }
            if (t2 >= 0 && t2 <= 1) {
                if (x2Root > minX && x2Root < maxX && y2Root > minY && y2Root < maxY) {
                    allPoints.add(new Point(x2Root, y2Root));
                }
            }

        } catch(Exception ignored) {//Ha. Get your problems ignored.

        }
        return allPoints;
    }
}
