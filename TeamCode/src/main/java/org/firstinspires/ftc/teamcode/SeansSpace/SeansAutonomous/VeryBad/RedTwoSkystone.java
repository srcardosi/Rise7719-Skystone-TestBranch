package org.firstinspires.ftc.teamcode.SeansSpace.SeansAutonomous.VeryBad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;
import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving.AutonomousPathing;

/**
 * Created by Sean Cardosi on 2019-12-14.
 */
@TeleOp(name = "Red Two Stone",group = "MkI Red Auto")
public class RedTwoSkystone extends LinearOpMode {

    AutonomousPathing path;
    String pose;
    ElapsedTime etime = new ElapsedTime();
    double tileWidth = 24;
    double stoneLength = 8;
    Point robotStartPose = new Point(9,9+tileWidth);

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() {
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents mech = new RobotComponents(hardwareMap);
        enc.init();
        path = new AutonomousPathing(hardwareMap);
        path.init();

        while (!this.isStarted) {
            path.initSearch(telemetry);
        }

        path.stopSearch();

        //--------------------------=+(Start Here)+=--------------------------\\
        waitForStart();

        pose = path.findPath();





        //--------------------------=+(Right)+=--------------------------\\






        if (pose.equals("1or4")) {//Right

            //----=+(First)+=----\\
            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 1*stoneLength + stoneLength/2);
            double exactAngleToStone = Math.toDegrees(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            double angleToStone = (Math.floor(exactAngleToStone) + Math.ceil(exactAngleToStone)) / 2;

            //Calculate the distance to the stone
            double exactDistanceToStone = Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y);
            double distanceToStone = (Math.floor(exactDistanceToStone) + Math.ceil(exactAngleToStone)) / 2;

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone

            //----=+(Second)+=----\\
            enc.steeringDrive(-(3*tileWidth + 4),false,false);//Goal here is to straighten by hitting the wall.
            enc.steeringDrive(-16,false,true);//Push other stones out of the way.
            mech.ejectStone();
            waitFor(1000);//Intake stone
            enc.steeringDrive(4,false,false);//Get the stone
            enc.steeringDrive(16,false,true);//Get clear of the bridge
            enc.steeringDrive(3*tileWidth,false,false);//Get under bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone





            //--------------------------=+(Middle)+=--------------------------\\






        } else if (pose.equals("2or5")) {//Middle

            //----=+(First)+=----\\
            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 1*stoneLength + stoneLength/2);
            double exactAngleToStone = Math.toDegrees(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            double angleToStone = (Math.floor(exactAngleToStone) + Math.ceil(exactAngleToStone)) / 2;

            //Calculate the distance to the stone
            double exactDistanceToStone = Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y);
            double distanceToStone = (Math.floor(exactDistanceToStone) + Math.ceil(exactAngleToStone)) / 2;

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone

            //----=+(Second)+=----\\
            enc.steeringDrive(-(3*tileWidth + 4),false,false);//Goal here is to straighten by hitting the wall.
            enc.gyroTurn(enc.TURN_SPEED,-90);//Face wall
            enc.steeringDrive(-8,false,false);//Back up to get clear of middle stone
            enc.steeringDrive(16,false,true);//Push wrong stones out of way.
            mech.ejectStone();
            waitFor(1000);//Pick up stone
            enc.steeringDrive(4,false,false);//Go forward to get stone.
            enc.steeringDrive(-16,false,true);//Strafe to get clear of bridge
            enc.steeringDrive(-3*tileWidth,false,false);//Go under bridge
            enc.gyroTurn(enc.TURN_SPEED,90);//Turn towards foundation
            mech.intakeStone();
            waitFor(1000);//Eject stone and hope you are over the line





            //--------------------------=+(Left)+=--------------------------\\






        } else if (pose.equals("3or6")) {//Left

            //----=+(First)+=----\\
            enc.steeringDrive(4,false,false);//Get Clear of the Wall
            robotStartPose.x += 4;//Add the change to the robot pose

            //Calculate the angle to turn to face the stone... this is for speed
            Point skystonePose = new Point(2*tileWidth,tileWidth + 1*stoneLength + stoneLength/2);
            double exactAngleToStone = Math.toDegrees(Math.atan2(skystonePose.y-robotStartPose.y,skystonePose.x-robotStartPose.x));
            double angleToStone = (Math.floor(exactAngleToStone) + Math.ceil(exactAngleToStone)) / 2;

            //Calculate the distance to the stone
            double exactDistanceToStone = Math.hypot(skystonePose.x-robotStartPose.x,skystonePose.y-robotStartPose.y);
            double distanceToStone = (Math.floor(exactDistanceToStone) + Math.ceil(exactAngleToStone)) / 2;

            enc.gyroTurn(enc.TURN_SPEED, angleToStone);//Point to the stone
            mech.intakeStone();
            waitFor(1000);//Fold out intake
            mech.ejectStone();
            waitFor(1000);//Enable intake
            enc.steeringDrive(distanceToStone+4,false,false);//Get the stone
            enc.steeringDrive(-5,false,false);//Back up to stay clear of bridge
            enc.gyroTurn(enc.TURN_SPEED,90);//Turn towards bridge
            enc.steeringDrive(35,false,false);//Go past bridge
            mech.intakeStone();
            waitFor(1000);//Eject the stone
            enc.steeringDrive(-2,false,false);//Park

            //----=+(Second)+=----\\
            enc.steeringDrive(-(3*tileWidth + 4),false,false);//Goal here is to straighten by hitting the wall.
            enc.gyroTurn(enc.TURN_SPEED,-90);//Face wall
            enc.steeringDrive(-16,false,false);//Back up to get clear of right stone
            enc.steeringDrive(16,false,true);//Push wrong stones out of way.
            mech.ejectStone();
            waitFor(1000);//Pick up stone
            enc.steeringDrive(4,false,false);//Go forward to get stone.
            enc.steeringDrive(-16,false,true);//Strafe to get clear of bridge
            enc.steeringDrive(-3*tileWidth,false,false);//Go under bridge
            enc.gyroTurn(enc.TURN_SPEED,90);//Turn towards foundation
            mech.intakeStone();
            waitFor(1000);//Eject stone and hope you are over the line





            //--------------------------=+(Unknown)+=--------------------------\\







        } else if (pose.equals("unknown")) {//Blind grab two stones

            //----=+(Blind)+=----\\
            enc.steeringDrive(4,false,false);//Get clear of wall
            enc.gyroTurn(enc.TURN_SPEED,-90);//Turn towards other wall
            enc.steeringDrive(-5,false,true);//Line up with wall
            enc.steeringDrive(-15,false,false);//Back up to be in a position to strafe and push stone
            enc.steeringDrive( 44.3, false, true);
            mech.intake.setPower(-1);
            waitFor(1000);
            mech.ejectStone();
            waitFor(500);
            enc.steeringDrive(4, false, false);
            mech.stopStone();
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-33,false,false);
            enc.gyroTurn(enc.TURN_SPEED,0);
            mech.intakeStone();
            waitFor(1000);
            enc.gyroTurn(enc.TURN_SPEED,-90);
            enc.steeringDrive(35,false,false);
            enc.steeringDrive(18,false,true);
            mech.ejectStone();
            waitFor(1000);
            enc.steeringDrive(8, false, false);
            mech.stopStone();
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-50,false,false);
            enc.gyroTurn(enc.TURN_SPEED,0);
            mech.intakeStone();
            waitFor(1000);
            enc.gyroTurn(enc.TURN_SPEED,-90);
            enc.steeringDrive(20,false,false);
        }
        AutoTransitioner.transitionOnStop(this,"MkI TeleOp");//Transition
    }
}
