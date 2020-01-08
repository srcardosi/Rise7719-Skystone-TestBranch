package org.firstinspires.ftc.teamcode.SeansSpace.ArchivedProjects.PositionIntegration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Archived;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Sean Cardosi on 10/21/2019.
 * Just like before, this isn't going to work.
 */
//@Archived
@Disabled
@TeleOp(name = "PreMadeIMUIntegration", group = "Integration")
public class PositionBNO055Function extends OpMode {

    private BNO055IMU gyro;
    private Orientation angle;
    NaiveAccelerationIntegrator integrator;
    Acceleration acceleration;

    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled      = false;
        param.loggingTag          = "IMU";

        gyro.initialize(param);
        angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyro.startAccelerationIntegration(new Position(), new Velocity(), 250);

        integrator.initialize(param,  new Position(),new Velocity());

    }

    public void loop() {

        acceleration.unit = DistanceUnit.CM;
        integrator.getAcceleration().unit = DistanceUnit.CM;
        integrator.getPosition().unit = DistanceUnit.CM;
        integrator.getVelocity().unit = DistanceUnit.CM;

        integrator.update(integrator.getAcceleration());

        telemetry.addData("GyroPosition(x,y)","(%d,%d)", gyro.getPosition().x, gyro.getPosition().y);

        telemetry.addData("NavigationPosition(x,y)","(%d,%d)", integrator.getPosition().x, integrator.getPosition().y);
    }
}
