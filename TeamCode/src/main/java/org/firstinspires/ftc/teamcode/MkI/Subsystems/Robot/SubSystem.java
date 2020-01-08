package org.firstinspires.ftc.teamcode.MkI.Subsystems.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Stolen from KNO3 Robotics' Jaxon Brown XOXO
 */

public abstract class SubSystem {
    /**
     * The robot this subsystem is a part of.
     */
    protected Robot robot;

    /**
     * The initialization routine of this subsystem. Cache stuff from the hardware map during this routine.
     */
    public abstract void init();

    /**
     * Handle driver controlled updates. You can fetch the gamepads with {@Code robot.gamepadX}.
     */
    public abstract void handle();

    /**
     * Stop everything on the robot.
     */
    public abstract void stop();

    /**
     * Construct a subsystem with the robot it applies to.
     * @param robot
     */
    public SubSystem(Robot robot) {
        this.robot = robot;
    }

    protected HardwareMap hardwareMap() {
        return robot.hardwareMap;
    }

    protected Gamepad gamepad1() {
        return robot.gamepad1;
    }

    protected Gamepad gamepad2() {
        return robot.gamepad2;
    }

    protected Telemetry telemetry() {
        return robot.telemetry;
    }
}