package org.firstinspires.ftc.teamcode.SeansSpace.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansSpace.RevExtensions.ExpansionHubEx;

/**
 * Created by Sean Cardosi on 2019-12-01.
 */
@Disabled

@TeleOp(name = "Hub LED ColoR Changer",group = "Diagnostics")
public class HubLEDColor extends LinearOpMode
{
    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        String header =
                "**********************************\n" +
                "      Expansion Hub LED Color     \n" +
                "**********************************\n";
        telemetry.addLine(header);

        /*
         * Setting ExpansionHub LED color
         */
        expansionHub.setLedColor(255, 0, 0);
        telemetry.addLine("Setting Hub LED color");

//        /*
//         * Setting ExpansionHub I2C bus speed
//         */
//        expansionHub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
//        telemetry.addLine("Setting speed of all I2C buses");

        telemetry.update();

        waitForStart();
        while (opModeIsActive());
    }
}