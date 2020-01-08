package org.firstinspires.ftc.teamcode.SeansSpace.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansSpace.RevExtensions.ExpansionHubEx;

/**
 * Created by Sean Cardosi on 2019-12-01.
 */
@Disabled

@TeleOp(name = "Hub Hardware and Firmware Monitor",group = "Diagnostics")
public class HubHardwareFirmwareMonitor extends LinearOpMode {
    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Before init() was called on this user code, REV Extensions 2
         * was notified via OpModeManagerNotifier.Notifications and
         * it automatically took care of initializing the new objects
         * in the hardwaremap for you. Historically, you would have
         * needed to call RevExtensions2.init()
         */
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        /*
         * ------------------------------------------------------------------------------------------------
         * HW/SW info
         * ------------------------------------------------------------------------------------------------
         */

        String header =
                "**********************************\n" +
                "       Firmware and Hardware      \n" +
                "**********************************\n";
        telemetry.addLine(header);

        telemetry.addData("Firmware", expansionHub.getFirmwareVersion());
        telemetry.addData("Hardware Revision", expansionHub.getHardwareRevision());
        telemetry.update();

        waitForStart();
        while (opModeIsActive());
    }
}