package org.firstinspires.ftc.teamcode.SeansSpace.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansSpace.RevExtensions.ExpansionHubEx;

/**
 * Created by Sean Cardosi on 2019-12-01.
 */
@Disabled
@TeleOp(name = "Voltage Monitor",group = "Diagnostics")
public class BatteryVoltageMonitor extends OpMode {

    ExpansionHubEx expansionHub;

    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }

    @Override
    public void loop() {

        /*
         * ------------------------------------------------------------------------------------------------
         * Voltage monitors
         * ------------------------------------------------------------------------------------------------
         */

        String header =
                "**********************************\n" +
                "         VOLTAGE MONITORS         \n" +
                "**********************************\n";
        telemetry.addLine(header);

        telemetry.addData("5v monitor (Phone Voltage)", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
        telemetry.addData("12v monitor (Battery Voltage)", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
        telemetry.update();

    }
}
