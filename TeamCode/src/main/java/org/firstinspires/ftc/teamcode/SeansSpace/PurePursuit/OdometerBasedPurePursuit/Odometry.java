package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit;

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
 * Created by Sean Cardosi on 10/17/2019
 * Class used for finding the robots current angle and location on the field.
 */
public class Odometry {

    private final DcMotor lo, ro;

    public BNO055IMU gyro;
    private Orientation angles;

    private Telemetry telemetry;

    double xLocation = 0.0;
    double yLocation = 0.0;
    private double distance = 0.0;
    private double changeRight = 0.0;
    private double changeLeft = 0.0;
    private double previousRightValue = 0.0;
    private double previousLeftValue = 0.0;

    //TODO: Update these values for odometers
    private double COUNTS_PER_REV = 537.6;
    private double EXTERNAL_GEAR_RATIO = 0.78125;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    double COUNTS_PER_INCH = ((COUNTS_PER_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));


    Odometry(HardwareMap hardwareMap, Telemetry tel) {

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

        lo = hardwareMap.dcMotor.get("lo");
        lo.setDirection(DcMotor.Direction.FORWARD);
        lo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ro = hardwareMap.dcMotor.get("ro");
        ro.setDirection(DcMotor.Direction.REVERSE);
        ro.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        lo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ro.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /*
     * Updates the previous encoder values..
     */
    private void previousValues() {

        previousRightValue = ro.getCurrentPosition();
        previousLeftValue = lo.getCurrentPosition();

    }

    /*
     * Finds the robots (x,y) location using the previous encoder values and the robot heading.
     */
    void updateLocation() {


        loop();

        changeRight = ro.getCurrentPosition() - previousRightValue;
        changeLeft = lo.getCurrentPosition() - previousLeftValue;

        distance = ((changeRight + changeLeft) / 2.0);
        xLocation += (distance * Math.cos((getRawHeading())));
        yLocation += (distance * Math.sin((getRawHeading())));

        telemetry.addData("x,y ", "%f,%f",xLocation/COUNTS_PER_INCH, yLocation/COUNTS_PER_INCH);
        telemetry.addData("Heading", Math.toDegrees(getRawHeading()));

        previousValues();
    }


    /**
     * Finds the robot's current heading on a scale equivalent with atan2.
     * @return Return the robot's angle
     */
    double getRawHeading() {

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
