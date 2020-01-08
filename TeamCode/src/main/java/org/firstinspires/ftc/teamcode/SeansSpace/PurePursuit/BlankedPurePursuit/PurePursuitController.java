package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.BlankedPurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;

import java.util.ArrayList;

import static java.lang.Math.ceil;
import static java.lang.Math.hypot;
import static java.lang.Math.*;

/**
 * Created by Sean Cardosi on 2019-12-23.
 */
public class PurePursuitController {

    //TODO: Change this to GlobalOdometry if you are using odometers for localization.
    private DriveWheelGlobalOdometry odometry;

    private DcMotor lf, lr, rf, rr;
    private double lfPower = 0.0;
    private double lrPower = 0.0;
    private double rfPower = 0.0;
    private double rrPower = 0.0;
    private int currentPointIndex = 1;
    private int nextPointIndex = 2;
    private double xPower = 0;
    private double yPower = 0;
    private double turnPower = 0;


    //------------------------------------=+(Debugging)+=--------------------------------------\\
    /**
     * Debugging.
     * @param hardwareMap hardwareMap mapping the hardware configuration
     * @param telemetry telemetry for displaying data to the Driver Station
     */
    public PurePursuitController(HardwareMap hardwareMap, Telemetry telemetry) {
        odometry = new DriveWheelGlobalOdometry(hardwareMap,telemetry);
        //configuring the components
        lr = hardwareMap.dcMotor.get("leftB");
        lf = hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rr = hardwareMap.dcMotor.get("rightB");
        rf = hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: RUN_WITHOUT_ENCODER if you are using odometers for localization.
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //-----------------------------------------------------------------------------------------\\


    //----------------------------------=+(Initialization)+=-----------------------------------\\
    /**
     * Initialization.
     */
    public void init() {
        odometry.init();
        //The first point should be the robots starting location.
        // An index of 0 would stop the robot from moving as it already reached its target.
        currentPointIndex = 1;
        nextPointIndex = 2;
        xPower = 0;
        yPower = 0;
        turnPower = 0;
        lfPower = 0.0;
        lrPower = 0.0;
        rfPower = 0.0;
        rrPower = 0.0;
    }
    //-----------------------------------------------------------------------------------------\\


    //------------------------------=+(Robot Pathing Functions)+=------------------------------\\
    /**
     * Gets the curvature at a certain point.
     * @param path The array of points that make a path.
     * @param pointIndex Index of the point to get the curvature at
     * @return Returns the paths curvature.
     */
    private double getCurvature(ArrayList<Point> path, int pointIndex) {

        Point P = path.get(pointIndex-1);//x1,y1
        Point Q = path.get(pointIndex);//x2,y2
        Point R = path.get(pointIndex+1);//x3,y3

        double x1 = P.x;
        double y1 = P.y;
        double x2 = Q.x;
        double y2 = Q.y;
        double x3 = R.x;
        double y3 = R.y;

        if (x1 == x2) {//No divide by 0 errors
            x1 += 0.001;
        }

        //Dumb equations that nobody cares about.
        double k1 = 0.5 * (pow(x1,2) + pow(y1,2) - pow(x2,2) - pow(y2,2)) / (x1 - x2);
        double k2 = (y1 - y2) / (x1 - x2);
        double b = 0.5 * (pow(x2,2) - 2 * x2 * k1 + pow(y2,2) - pow(x3,2) + 2 * x3 * k1 - pow(y3,2)) / (x3 * k2 - y3 + y2 - x2 * k2);
        double a = k1 - k2 * b;
        double r = sqrt(pow(x1 - a,2) + pow(y1 - b,2));

        return 1/r;//curvature
    }

    /**
     * Applies the appropriate powers to the drive wheels.
     */
    private void ApplyPower() {

        //Shorten the variables for ease.
        double x = xPower;
        double y = yPower;
        double r = turnPower;

        //Power formulas for mechanum wheels.
        lfPower = x + y + r;
        rfPower = x - y - r;
        lrPower = x - y + r;
        rrPower = x + y - r;

        //find the maximum of the powers
        double lfmaxRawPower = Math.abs(lfPower);

        if(Math.abs(lfPower) > lfmaxRawPower){ lfmaxRawPower = Math.abs(lfPower);}
        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(lfmaxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/lfmaxRawPower;
        }

        //Scale down the powers if necessary.
        lfPower *= scaleDownAmount;
        lrPower *= scaleDownAmount;
        rrPower *= scaleDownAmount;
        rfPower *= scaleDownAmount;

        //Finally set the powers.
        lf.setPower(lfPower);
        lr.setPower(lrPower);
        rf.setPower(rfPower);
        rr.setPower(rrPower);
    }

    /**
     * Injects more points into a given array of points to increase accuracy.
     * @param pathPoints The array of a standard user created path.
     * @param spacing The desired spacing between points in inches.
     * @return Returns a new array of points that includes the injected points and the original points
     */
    private ArrayList<Point> injectPoints(ArrayList<Point> pathPoints, double spacing) {

        ArrayList<Point> newPoints = new ArrayList<>();

        //For all the points in the path.
        for (int i=0; i<pathPoints.size()-1; i++) {

            //Define a new vector
            Point startPoint = pathPoints.get(i);
            Point endPoint = pathPoints.get(i+1);

            //Find the magnitude of the vector.
            double magnitude = hypot(endPoint.x - startPoint.x, endPoint.y-startPoint.y);

            //Find how many points can fit in between the start point and end point.
            double pointsThatFit = ceil(magnitude / spacing);

            //Define the vector as a point.
            Point vector = new Point(endPoint.x-startPoint.x,endPoint.y-startPoint.y);

            //Normalize the vector.
            Point normalizedVector = new Point(vector.x/magnitude,vector.y/magnitude);

            //Calculate the x and y values to increment by for the injected points.
            normalizedVector.x *= (magnitude / pointsThatFit);
            normalizedVector.y *= (magnitude / pointsThatFit);

            //Add the injected points into the array for this segment.
            for (int j=0; j<pointsThatFit; j++) {
                newPoints.add(new Point(startPoint.x+normalizedVector.x*j,startPoint.y+normalizedVector.y*j));
            }
            newPoints.add(endPoint);//Add the last point to the new path that includes the injected points.
        }
        return newPoints;
    }

    /**
     * Smooths the pure pursuit path using a Bezier Curve.
     * @param controlPoints The array of points to base the curve off of.
     * @param precision Between 0 and 1. Lower is more precise. Realistically it should be between 0.01 and 0.1.
     * @return Returns a new array of points for the robot to follow.
     */
    private ArrayList<Point> BezierCurve(ArrayList<Point> controlPoints, double precision) {

        ArrayList<Point> smoothPath = new ArrayList<>();
        double t = 0;//The t value starts at 0.
        double n = controlPoints.size() - 1;//The n-degree is given by the number of points-1.
        double curveX = 0;//X-Value on the curve
        double curveY = 0;//Y-Value on the curve

        while (t <= 1) {//t is [0,1]

            curveX = 0;
            curveY = 0;

            //This for statement is used as a sigma
            for (int i=0; i<=n; i++) {

                //P(t)= Sigma (n i) * (1-t)^(n-i) * t^i * P_i
                curveX += biCoef(n,i) * pow((1-t),(n-i)) * pow(t,i) * controlPoints.get(i).x;//X-Value
                curveY += biCoef(n,i) * pow((1-t),(n-i)) * pow(t,i) * controlPoints.get(i).y;//Y-Value
            }

            smoothPath.add(new Point(curveX,curveY));//Add the point on the Bezier Curve into an array
            t += precision;//Increment t by a small amount to find the next point on the Bezier Curve
        }
        smoothPath.add(new Point(controlPoints.get(controlPoints.size()-1).x,controlPoints.get(controlPoints.size()-1).y));//Add the last point on the path.
        return smoothPath;
    }

    /**
     * Builds the path for the robot to follow.
     * @param waypoints An array of waypoints to form an outline of the path for the robot to follow.
     * @param spacing The desired spacing between the points that will be injected into the path.
     * @param precision How closely spaced the points on the curved line should be.
     *                  This is not in inches. It is a number between 0 and 0.5 and
     *                  should be somewhere around 0.05. The lower the value, the more accurate the
     *                  path will be, but it also causes a higher wait time when the method is called.
     *                  The wait time will vary based on the loop speed.
     */
    public void buildPath(ArrayList<Point> waypoints, double spacing, double precision) {

        //Inject points into the current path.
        ArrayList<Point> injectedPointPath = new ArrayList<>(injectPoints(waypoints,spacing));

        //Smooth the path using a bezier curve
        ArrayList<Point> robotPath = new ArrayList<>(BezierCurve(injectedPointPath,precision));

        //Replace the points in the current array with the new smooth path.
        waypoints.clear();
        waypoints.addAll(robotPath);
    }

    /**
     * Uses an Adaptive Pure Pursuit type of algorithm to make the robot follow a path.
     * @param path A previously built path for the robot to follow.
     * @param speed The speed for the robot to follow the path at.
     * @param followAngle The angle the robot should follow the path at. An angle of 0 is forward,
     *                    and and angle of 180 or -180 is backwards. You can also
     *                    use other angles like 30 degrees. Following paths sideways is not supported.
     * TODO:              Use the distance from the target point as error for the PID Controller.
     *                    Fix the follow angle so you can follow the path at a new angle.
     */
    public void followPath(ArrayList<Point> path, double speed, double followAngle) {

        //Reset the pointIndex
        currentPointIndex = 1;
        nextPointIndex = 2;

        //TODO: Use PID so you don't have to do this.
        while (nextPointIndex != path.size()-1) {

            //Define some variables
            Point robot = new Point(odometry.xLocation, odometry.yLocation);
            Point currentPoint = new Point(path.get(currentPointIndex).x, path.get(currentPointIndex).y);
            Point nextPoint = new Point(path.get(nextPointIndex).x, path.get(nextPointIndex).y);

            //Get the curvature to find whihc way the path curves.
            double curvature = getCurvature(path, currentPointIndex);
            int curvatureDirection = (int) (signum(curvature));

            //Calculate distances and angles until the points.
            double distanceToCurrentPoint = hypot(currentPoint.x - robot.x, currentPoint.y - robot.y);
            double distanceToNextPoint = hypot(nextPoint.x - robot.x, nextPoint.y - robot.y);
            double angleToPoint = (atan2(nextPoint.y - robot.y, nextPoint.x - robot.x));
            double relativeAngleToPoint = AngleUnit.normalizeRadians(angleToPoint - odometry.getRawHeading());

            //Calculate the distance to the target point.
            double xToPoint = cos(relativeAngleToPoint) * distanceToNextPoint;
            double yToPoint = sin(relativeAngleToPoint) * distanceToNextPoint;

            //Provide power.
            xPower = (xToPoint / (abs(xToPoint) + abs(yToPoint))) * speed;
            yPower = (yToPoint / (abs(xToPoint) + abs(yToPoint))) * speed;
            turnPower = (-Range.clip((relativeAngleToPoint/* + (curvatureDirection) * toRadians(followAngle)*/) / Math.toRadians(30), -1, 1)) * speed;

            //Update the location and power cycle.
            odometry.updateLocation();
            ApplyPower();

            //If nessecary move on the next point on the path.
            if (distanceToNextPoint < distanceToCurrentPoint) {
                currentPointIndex++;
                nextPointIndex++;
            }
        }
        //When done we need to stop.
        xPower = 0;
        yPower = 0;
        turnPower = 0;
        ApplyPower();
    }
    //-----------------------------------------------------------------------------------------\\


    //----------------------------------=+(Array Functions)+=----------------------------------\\
    /**
     * Simple method to add a point to an array.
     * @param waypoints The array of points
     * @param points Points to add into the array.
     */
    public void addWaypoints(ArrayList<Point> waypoints, Point... points) {
        for (Point p : points) {
            waypoints.add(new Point(p.x,p.y));
        }
    }

    /**
     * User friendly method to clear the waypoints out of an array.
     * @param waypoints The array of waypoints.
     */
    public void clearWaypoints(ArrayList<Point> waypoints) {
        waypoints.clear();
    }
    //-----------------------------------------------------------------------------------------\\


    //----------------------------------=+(Math Functions)+=-----------------------------------\\
    /**
     * Binomial coefficient formula. m >= n
     * @param m Pretty straight forward given m is the top number in the parenthesis.
     * @param n Pretty straight forward given n is the top number in the parenthesis.
     * @return Returns the binomial coefficient of the inputs.
     */
    private double biCoef(double m, double n) {
        return factorial(m) / (factorial(n) * factorial((m-n)));//m! / (n!(m-n)!)
    }

    /**
     * Calculates the factorial of a given number.
     * @param number The number to find the factorial of.
     * @return Returns the factorial. Keep in mind 0! is 1.
     */
    private int factorial(double number) {
        int theFactorial = 1;
        for (int i=1; i<=number; i++) {
            theFactorial *= i;
        }
        if (theFactorial == 0) {
            return 1;//0!=1
        } else {
            return theFactorial;
        }
    }
    //-----------------------------------------------------------------------------------------\\
}
