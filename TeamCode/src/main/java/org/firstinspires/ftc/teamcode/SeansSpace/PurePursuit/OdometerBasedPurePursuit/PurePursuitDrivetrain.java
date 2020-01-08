package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Sean Cardosi.
 * PurePursuitDrivetrain is the drivetrain class to only be used with Pure Pursuit.
 */
public class PurePursuitDrivetrain {

    private static DcMotor lf, lr, rf, rr;
    private double lfPower = 0.0;
    private double lrPower = 0.0;
    private double rfPower = 0.0;
    private double rrPower = 0.0;


    public PurePursuitDrivetrain(final HardwareMap _hardwareMap) {

        //configuring the components
        lr = _hardwareMap.dcMotor.get("leftB");
        lf = _hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rr = _hardwareMap.dcMotor.get("rightB");
        rf = _hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        lfPower = 0.0;
        lrPower = 0.0;
        rfPower = 0.0;
        rrPower = 0.0;
    }

    void ApplyPower() {


        double x = PurePursuitMovement.movementX;
        double y = PurePursuitMovement.movementY;
        double r = PurePursuitMovement.movementTurn;

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

        lfPower *= scaleDownAmount;
        lrPower *= scaleDownAmount;
        rrPower *= scaleDownAmount;
        rfPower *= scaleDownAmount;

        lf.setPower(lfPower);
        lr.setPower(lrPower);
        rf.setPower(rfPower);
        rr.setPower(rrPower);

    }
}
