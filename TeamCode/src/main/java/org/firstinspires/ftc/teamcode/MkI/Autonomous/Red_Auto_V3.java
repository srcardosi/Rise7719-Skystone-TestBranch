package org.firstinspires.ftc.teamcode.MkI.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.MkI.TeleOp.MkITeleOp;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.GGOpenCV;
//import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.GGSkystoneDetector;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.VisionSystem;
//import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving.AutonomousPathing;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
//import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.Drivetrain;
//import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;
//import org.firstinspires.ftc.teamcode.MkI.Subsystems.VisionTargeting.SkystoneDetectionPhone;


@Autonomous (name = "Red Auto Both", group = "MkI Blue Auto")
public class Red_Auto_V3 extends LinearOpMode{

//    GGSkystoneDetector vis;
//    GGOpenCV detector;
    ElapsedTime etime = new ElapsedTime();
    private double posit = 0;

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode(){

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents mech = new RobotComponents(hardwareMap);
        GGOpenCV detector = new GGOpenCV(GGOpenCV.Cam.PHONE, hardwareMap);
        detector.startCamera();
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        enc.init();



        while (!this.isStarted){
            if (detector.found()){
                telemetry.addData("SS Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);

                posit = detector.detector.foundRectangle().x;
                telemetry.addData("Position: ", posit);

            } else {
                telemetry.addData("SS not found.", "");
            }

            telemetry.update();
        }

        detector.stopLook();

//      position 3 and 6
        if((posit>=225&&posit<=290)||(posit>=35&&posit<65)){


            enc.steeringDrive(10,false,true);
            enc.steeringDrive(44.3,false,false);
            enc.steeringDrive(4, false, false);
            waitFor(1000);
            mech.intakeStone();
            waitFor(500);
            enc.steeringDrive(18,false,true);
            enc.steeringDrive(-33,false,false);
            waitFor(1000);
            mech.ejectStone();



        }

 //     position 1 and 4
        if ((posit>=170&&posit<=220)||(posit>=1&&posit<=30)){


            enc.steeringDrive(20,false,true);
            enc.steeringDrive(44.3,false,false);
            enc.steeringDrive(4, false, false);
            waitFor(1000);
            mech.intakeStone();
            waitFor(500);
            enc.steeringDrive(18,false,true);
            enc.steeringDrive(-43,false,false);
            waitFor(1000);
            mech.ejectStone();


        }
//         position 2 and 5
        if ((posit>=110&&posit<=169)||(posit==0)){


            enc.steeringDrive(14,false,true);
            enc.gyroTurn(enc.TURN_SPEED,-90);
            enc.steeringDrive(38.3,false,true);
            waitFor(1000);
            mech.ejectStone();
            waitFor(1500);
            enc.steeringDrive(6, false, false);
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-53,false,false);
            enc.gyroTurn(enc.TURN_SPEED,90);
            waitFor(1000);
            mech.intakeStone();
            waitFor(2000);
            enc.steeringDrive(-7,false,false);


        }


    }










}








