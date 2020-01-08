package org.firstinspires.ftc.teamcode.SeansSpace.SeansAutonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving.SeansDefaultEncoderLibrary;

/**
 * Created by Sean Cardosi on 2019-12-16.
 */
@Autonomous(name = "Default Encoder Test", group = "SeansSpace")
public class DefaultEncoderTest extends OpMode {

    SeansDefaultEncoderLibrary enc;

    public BNO055IMU gyro;
    public Orientation gyro_angle;


    @Override
    public void init() {

        enc = new SeansDefaultEncoderLibrary(hardwareMap);
        enc.init();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(param);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            enc.drive(24, 1);
        }
        if (gamepad1.a) {
            enc.strafe(24,1);
        }
        if (gamepad1.b) {
            enc.turn(90,1);
        }
        gyro_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading",gyro_angle.firstAngle);
    }
}
