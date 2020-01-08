package org.firstinspires.ftc.teamcode.MkI.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SrvoTest", group = "testing")
public class ServoTest extends OpMode {



    private Servo lgrab , rgrab;
    @Override
    public void init() {

        lgrab = hardwareMap.servo.get("lgrab");
        rgrab = hardwareMap.servo.get("rgrab");
        rgrab.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            lgrab.setPosition(0);
            rgrab.setPosition(0);
        }
        if (gamepad1.y) {
            lgrab.setPosition(1);
            rgrab.setPosition(1);
        }
        if (gamepad1.a) {
            lgrab.setPosition(-1);
            rgrab.setPosition(-1);
        }
    }
}
