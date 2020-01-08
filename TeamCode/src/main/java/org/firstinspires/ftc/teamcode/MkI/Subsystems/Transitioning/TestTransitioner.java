package org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * Created by seancardosi on 9/22/19.
 */
@Autonomous(name = "TestTransitioner", group = "MemeMachine")
public class TestTransitioner extends LinearOpMode {//test OpMode to automatically switch from autonomous to TeleOp


    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initializing Here", true);
        telemetry.update();


        waitForStart();

        AutoTransitioner.transitionOnStop(this, "SeanTeleOp");

    }
}