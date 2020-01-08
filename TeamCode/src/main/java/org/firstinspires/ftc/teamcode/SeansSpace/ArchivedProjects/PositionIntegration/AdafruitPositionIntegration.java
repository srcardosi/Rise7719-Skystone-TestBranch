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
 * Created by Sean Cardosi on 10/24/2019.
 * Uses the Adafruit BNO055 Arduino code but translated to java.
 */
//@Archived
@Disabled
@TeleOp(name = "AdafruitIntegration",group = "Integration")
public class AdafruitPositionIntegration extends OpMode {

    private BNO055IMU gyro;
    private Orientation angle;
    private ElapsedTime elapsedTime;//To be more accurate
    private double LOOP_RATE_DELAY_MS = 20;//Avergae time it takes for the Opmode to loop once.
    //velocity = accel*dt (dt in seconds)
    //position = 0.5*accel*dt^2
    private double ACCEL_VEL_TRASITION = (LOOP_RATE_DELAY_MS) / 1000.0; //This is dt in seconds
    private double ACCEL_POS_TRANSTION = 0.5 * Math.pow(ACCEL_VEL_TRASITION, 2); //This is 0.5 * dt^2
    private double xPos = 0.0;
    private double yPos = 0.0;
    private double xPosElapsed = 0.0;
    private double yPosElapsed = 0.0;


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

        xPosElapsed += elapsedTime.seconds() * gyro.getAcceleration().xAccel;
        yPosElapsed += elapsedTime.seconds() * gyro.getAcceleration().yAccel;

        elapsedTime.reset();

        xPos += ACCEL_POS_TRANSTION * gyro.getAcceleration().xAccel; //This is 0.5 * dt^2 * accel
        yPos += ACCEL_POS_TRANSTION * gyro.getAcceleration().yAccel; //This is 0.5 * dt^2 * accel

        telemetry.addData("Position Using Average Loop Time: ","(%d:%d)", xPos, yPos);
        telemetry.addData("Position Using Elapse Time: ","(%d:%d)", xPosElapsed, yPosElapsed);
    }
}
