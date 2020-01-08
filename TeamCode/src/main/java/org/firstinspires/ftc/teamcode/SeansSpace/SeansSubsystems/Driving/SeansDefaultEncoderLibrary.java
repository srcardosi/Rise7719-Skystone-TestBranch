package org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;

/**
 * Created by Sean Cardosi on 2019-12-16.
 * Uses the built-in PID for movements. TODO: Add in a slip factor.
 */
public class SeansDefaultEncoderLibrary {

    DcMotor lf, lr, rf, rr;

    private double COUNTS_PER_MOTOR_REV = 537.6;
    private double EXTERNAL_GEAR_RATIO = 0.78125;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    private double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * PI));

    private int lfCountLast = 0;
    private int lrCountLast = 0;
    private int rfCountLast = 0;
    private int rrCountLast = 0;

    public SeansDefaultEncoderLibrary(HardwareMap hardwareMap) {

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
    }

    public void init() {

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lfCountLast = 0;
        lrCountLast = 0;
        rfCountLast = 0;
        rrCountLast = 0;

    }

    /**
     * Drive forwards or backwards using built-in PID.
     * @param inches Distance in inches.
     * @param speed Max speed to move at. You can use a high speed after the slip factor is implemented.
     */
    public void drive(double inches, double speed) {

        resetEncoders();

        int direction = (int)(signum(inches));
        double power = direction*speed;

        int lfTarget = (int)(inches*COUNTS_PER_INCH);
        int lrTarget = (int)(inches*COUNTS_PER_INCH);
        int rfTarget = (int)(inches*COUNTS_PER_INCH);
        int rrTarget = (int)(inches*COUNTS_PER_INCH);

        lf.setTargetPosition(lfTarget+lfCountLast);
        lr.setTargetPosition(lrTarget+lrCountLast);
        rf.setTargetPosition(rfTarget+rfCountLast);
        rr.setTargetPosition(rrTarget+rrCountLast);

        lf.setPower(power);
        lr.setPower(power);
        rf.setPower(power);
        rr.setPower(power);

        while (lf.isBusy() || lr.isBusy() || rf.isBusy() || rr.isBusy()) {
            //Don't stop until done. You could also put a state machine in here
            //such as moving a servo. The servo would move while the robot moves.
        }
    }

    /**
     * Strafe left or right using built-in PID.
     * Left is a negative distance. Right is positive distance.
     * @param distance Distance to strafe in inches.
     * @param speed Speed to strafe at.
     */
    public void strafe(double distance, double speed) {

        resetEncoders();

        int direction = (int)signum(distance);//-1 is left, 1 is right
        speed *= direction;

        int lfTarget = (int)(direction*abs(distance)*COUNTS_PER_INCH);
        int lrTarget = (int)(-direction*abs(distance)*COUNTS_PER_INCH);
        int rfTarget = (int)(-direction*abs(distance)*COUNTS_PER_INCH);
        int rrTarget = (int)(direction*abs(distance)*COUNTS_PER_INCH);

        lf.setTargetPosition(lfTarget+lfCountLast);
        lr.setTargetPosition(lrTarget+lrCountLast);
        rf.setTargetPosition(rfTarget+rfCountLast);
        rr.setTargetPosition(rrTarget+rrCountLast);

        lf.setPower(speed);
        lr.setPower(-speed);
        rf.setPower(-speed);
        rr.setPower(speed);

        while (lf.isBusy() || lr.isBusy() || rf.isBusy() || rr.isBusy()) {
            //Don't stop until done. You could also put a state machine in here
            //such as moving a servo. The servo would move while the robot moves.
        }
    }

    /**
     * Uses built-in PID to turn. Does not use the gyro.
     * Instead it calculates the distance each wheel
     * must travel to turn the specified angle.
     * @param angle Angle to turn in degrees.
     * @param speed Max speed to turn at. You can use a high speed after the slip factor is implemented.
     */
    public void turn(double angle, double speed) {

        resetEncoders();

        //TODO: Tune the trackwidth so the robot will turn accurately.
        //Formula for arc length given central angle: (angle/180)(2(pi)radius)
        //NOTE: It is angle/180 because the robot's angle is -180 to 180 not 0 to 360
        double trackWidth = 18;//The distance between the wheels of the robot: The diameter
        double radius = trackWidth/2;
        double arcLength = (angle/180) * (2 * PI * radius);

        int direction = (int)signum(arcLength);//-1 is left, 1 is right
        speed *= direction;

        int lfTarget = (int)(arcLength*COUNTS_PER_INCH);
        int lrTarget = (int)(arcLength*COUNTS_PER_INCH);
        int rfTarget = (int)(-arcLength*COUNTS_PER_INCH);
        int rrTarget = (int)(-arcLength*COUNTS_PER_INCH);

        lf.setTargetPosition(lfTarget+lfCountLast);
        lr.setTargetPosition(lrTarget+lrCountLast);
        rf.setTargetPosition(rfTarget+rfCountLast);
        rr.setTargetPosition(rrTarget+rrCountLast);

        lf.setPower(speed);
        lr.setPower(speed);
        rf.setPower(-speed);
        rr.setPower(-speed);

        while (lf.isBusy() || lr.isBusy() || rf.isBusy() || rr.isBusy()) {
            //Don't stop until done. You could also put a state machine in here
            //such as moving a servo. The servo would move while the robot moves.
        }
    }

    /**
     * Resets the encoders quickly without having to stop.
     */
    private void resetEncoders() {
        lfCountLast = lf.getCurrentPosition();
        lrCountLast = lr.getCurrentPosition();
        rfCountLast = rf.getCurrentPosition();
        rrCountLast = rr.getCurrentPosition();
    }

    /**
     * Stops all motors.
     */
    public void stopAllMotors() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}
