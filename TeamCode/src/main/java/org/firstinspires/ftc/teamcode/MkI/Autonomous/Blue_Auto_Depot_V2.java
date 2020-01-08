package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.VisionTargeting.SkystoneDetectionPhone;
@Disabled
@Autonomous(name = "Blue Depot",group = "MkI Blue Auto" )
public class Blue_Auto_Depot_V2 extends LinearOpMode{

    private boolean isSkystone;

    public void runOpMode() throws InterruptedException {

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents mech = new RobotComponents(hardwareMap);
        SkystoneDetectionPhone cam = new SkystoneDetectionPhone(hardwareMap, telemetry);

        enc.init();
        mech.init();
        cam.init();

        /* JORDAN
         * I don't think this is right because all you are doing is scanning for a skystone
         * once can then breaking. You don't have waitForStart(); so it will start as soon as you init.
         * I think you would want to go forward then strafe until a skystone is detected. I don't know
         * what you meant to do, but this program can't be used without a waitForStart(); so I am
         * disabling it. If this program is old, outdated, and if it will not be used then please
         * delete it.
         */

        //I also changed this from a while (true) to a do-while loop. In this case they do the same thing.
        do {

            isSkystone = cam.TFdetect(isSkystone);

        } while (!isStarted());


        enc.steeringDrive(6, true,false);

        enc.gyroTurn(enc.TURN_SPEED,-90);

        enc.steeringDrive(12, true, false);

        enc.gyroTurn(enc.TURN_SPEED,0);

        enc.steeringDrive(38, true, false);

        enc.steeringDrive(-6,true,false);

        enc.gyroTurn(enc.TURN_SPEED, 90);

        AutoTransitioner.transitionOnStop(this,"MkITeleOp");

    }
}
