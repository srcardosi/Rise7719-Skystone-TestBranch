package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;

@Autonomous (name = "PID Test", group = "Test Environment")
public class PID_Tune extends LinearOpMode {

    private ElapsedTime etime = new ElapsedTime();

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode(){

        //int position;
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents component = new RobotComponents(hardwareMap);

        enc.init();

        waitForStart();

//        telemetry.addLine("Forward no toggle");
//        telemetry.update();
//        enc.steeringDrive(24, false,false);
//        waitFor(1000);
//        telemetry.addLine("Backward no toggle");
//        telemetry.update();
//        enc.steeringDrive(-24, false,false);
//        waitFor(1000);
//        telemetry.addLine("Left no toggle");
//        telemetry.update();
//        enc.steeringDrive(-24, false,true);
//        waitFor(1000);
//        telemetry.addLine("Right no toggle");
//        telemetry.update();
//        enc.steeringDrive(24, false,true);
//        waitFor(1000);
        telemetry.addLine("Forward with toggle");
        telemetry.update();
        enc.steeringDrive(24, true,false);
        waitFor(1000);
        telemetry.addLine("Backward with toggle");
        telemetry.update();
        enc.steeringDrive(-24, true,false);
        waitFor(1000);
        telemetry.addLine("Left with toggle");
        telemetry.update();
        enc.steeringDrive(-24, true,true);
        waitFor(1000);
        telemetry.addLine("Right with toggle");
        telemetry.update();
        enc.steeringDrive(24, true,true);

//        AutoTransitioner.transitionOnStop(this,"MkITeleOp");
    }
}
