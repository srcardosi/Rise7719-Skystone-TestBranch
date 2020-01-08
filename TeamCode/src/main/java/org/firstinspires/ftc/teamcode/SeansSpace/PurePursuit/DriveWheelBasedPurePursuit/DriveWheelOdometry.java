package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * Created by Sean Cardosi on 11/6/2019
 * Class used for finding the robots current angle and location on the field.
 */
public class DriveWheelOdometry {

    private final DcMotor lf, lr, rf, rr;

    public BNO055IMU gyro;
    private Orientation angles;

    private Telemetry telemetry;

    public double xLocation = 0.0;
    public double yLocation = 0.0;
    private double distance = 0.0;
    private double changeRight = 0.0;
    private double changeLeft = 0.0;
    private double previousRightValue = 0.0;
    private double previousLeftValue = 0.0;
    private double COUNTS_PER_REV = 537.6;
    private double EXTERNAL_GEAR_RATIO = 0.78125;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    double COUNTS_PER_INCH = ((COUNTS_PER_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));


    public DriveWheelOdometry(HardwareMap hardwareMap, Telemetry tel) {

        //GYRO IS IN RADIANS FOR PURE PURSUIT
        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled       = true;
        param.useExternalCrystal   = true;
        param.mode                 = BNO055IMU.SensorMode.IMU;
        param.loggingTag           = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        lr = hardwareMap.dcMotor.get("leftB");
        lf = hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rr = hardwareMap.dcMotor.get("rightB");
        rf = hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = tel;
    }
    public void init () {

        xLocation = 0.0;
        yLocation = 0.0;
        distance = 0.0;
        changeRight = 0.0;
        changeLeft = 0.0;
        previousRightValue = 0.0;
        previousLeftValue = 0.0;

    }

    /*
     * Updates the previous encoder values..
     */
    private void previousValues() {

        previousRightValue = ((rr.getCurrentPosition() + rf.getCurrentPosition()) / 2.0);
        previousLeftValue = ((lr.getCurrentPosition() + lf.getCurrentPosition()) / 2.0);

    }

    /*
     * Finds the robots (x,y) location using the previous encoder values and the robot heading.
     */
    public void updateLocation() {


        loop();

        changeRight = ((rr.getCurrentPosition() + rf.getCurrentPosition()) / 2.0) - previousRightValue;
        changeLeft = ((lr.getCurrentPosition() + lf.getCurrentPosition()) / 2.0) - previousLeftValue;

        distance = ((changeRight + changeLeft) / 2.0);
        xLocation += (distance * Math.cos((getRawHeading())));
        yLocation += (distance * Math.sin((getRawHeading())));

        telemetry.addData("x,y ", "%f,%f",xLocation/COUNTS_PER_INCH, yLocation/COUNTS_PER_INCH);
        telemetry.addData("Heading", Math.toDegrees(getRawHeading()));


//        telemetry.addData("Distance",distance);
//        telemetry.addData("ChangeRight",changeRight);
//        telemetry.addData("ChnageLeft",changeLeft);
//        telemetry.addData("lf", lf.getCurrentPosition());
//        telemetry.addData("lr", lr.getCurrentPosition());
//        telemetry.addData("rf", rf.getCurrentPosition());
//        telemetry.addData("rr", rr.getCurrentPosition());

        previousValues();
    }


    /**
     * Finds the robot's current heading on a scale equivalent with atan2.
     * @return Return the robot's angle
     */
    public double getRawHeading() {

        double raw = angles.firstAngle + Math.toRadians(90);
        return AngleUnit.normalizeRadians(raw);// + Math.toRadians(90);
    }

    /**
     * This updates the robot's angles. Should be called every loop.
     */
    public void loop() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}