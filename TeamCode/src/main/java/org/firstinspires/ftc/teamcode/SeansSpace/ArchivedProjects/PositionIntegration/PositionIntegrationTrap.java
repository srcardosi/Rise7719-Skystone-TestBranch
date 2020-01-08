package org.firstinspires.ftc.teamcode.SeansSpace.ArchivedProjects.PositionIntegration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Archived;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Sean Cardosi on 10/22/2019.
 * Uses the formula for a trapezoid for integration. I don't think you can loop fast enough for this to work. If
 * this works, it will be more accurate than using the rectangle formula.
 */
//@Archived
@Disabled
@TeleOp(name = "Trapezoid Position Integration", group = "Integration")
public class PositionIntegrationTrap extends OpMode {

    private BNO055IMU gyro;
    private Orientation angle;
    private ElapsedTime elapsedTimeAccel;
    private ElapsedTime elapsedTimeVel;
    private double positionX = 0.0;
    private double positionY = 0.0;
    private double displacementX = 0.0;
    private double displacementY = 0.0;
    private double changeVelx = 0.0;
    private double changeVely = 0.0;
    private double velocityX = 0.0;
    private double velocityY = 0.0;
    private double prevVelx = 0.0;
    private double prevVely = 0.0;
    private double prevAccelx = 0.0;
    private double prevAccely = 0.0;
    private float loopNum = 0;

    @Override
    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public void loop() {

        if (loopNum == 0) {
            elapsedTimeAccel.reset();
            elapsedTimeVel.reset();
        }

        //a=∆v∆t so ∆v = a∆t
        changeVelx = ((prevAccelx + gyro.getAcceleration().xAccel) / 2) * (elapsedTimeAccel.seconds());
        changeVely = ((prevAccely + gyro.getAcceleration().yAccel) / 2) * (elapsedTimeAccel.seconds());

        prevAccelx = gyro.getAcceleration().xAccel;
        prevAccely = gyro.getAcceleration().yAccel;
        elapsedTimeAccel.reset();

        velocityX += changeVelx;
        velocityY += changeVely;

        // v = ∆s/∆t so ∆s = v∆t
        displacementX = ((prevVelx + velocityX) / 2) * (elapsedTimeVel.seconds());
        displacementY = ((prevVely + velocityY) / 2) * (elapsedTimeVel.seconds());

        prevVelx = velocityX;
        prevVely = velocityY;
        elapsedTimeVel.reset();

        positionX += displacementX;
        positionY += displacementY;

        telemetry.addData("Position(x,y)","(%d,%d)", positionX, positionY);
        telemetry.addData("Velocity(x,y)","(%d,%d)", velocityX, velocityY);
        telemetry.addData("Acceleration(x,y)","(%d,%d)", gyro.getAcceleration().xAccel, gyro.getAcceleration().yAccel);
        telemetry.addData("We are currently on loop: ", loopNum);

        if (loopNum == 0) {
            loopNum = 1;
        }
    }
}
