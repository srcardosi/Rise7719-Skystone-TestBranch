package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;

@Autonomous (name = "Red Auto Blocks", group = "MkI Red Auto")
public class Red_Auto_V1 extends LinearOpMode {


    ElapsedTime etime = new ElapsedTime();

    public void waitFor(int time) {
        time = time / 1000;
        etime.reset();
        while ((etime.time() < time) && (opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //int position;
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();
        RobotComponents mech = new RobotComponents(hardwareMap);

        waitForStart();


        //Position 1 (4 from wall)
        enc.steeringDrive( 44.3, false, true);
        mech.intake.setPower(-1);
        waitFor(1000);
        mech.ejectStone();
        waitFor(500);
        enc.steeringDrive(4, false, false);
        enc.steeringDrive(-18,false,true);
        enc.steeringDrive(-35,false,false);
        enc.gyroTurn(enc.TURN_SPEED,90);
        mech.intakeStone();
        waitFor(1000);
        enc.gyroTurn(enc.TURN_SPEED,0);
        enc.steeringDrive(37,false,false);
        enc.steeringDrive(19,false,true);
        mech.ejectStone();
        waitFor(1000);
        enc.steeringDrive(8, false, false);
        enc.steeringDrive(-18.5,false,true);
        enc.steeringDrive(-50,false,false);
        enc.gyroTurn(enc.TURN_SPEED,90);
        mech.intakeStone();
        waitFor(1000);
        enc.gyroTurn(enc.TURN_SPEED,0);
        enc.steeringDrive(22,false,false);


        AutoTransitioner.transitionOnStop(this,"MkITeleOp");
    }
}
