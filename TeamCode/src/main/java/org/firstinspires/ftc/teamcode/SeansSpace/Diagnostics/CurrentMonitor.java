package org.firstinspires.ftc.teamcode.SeansSpace.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansSpace.RevExtensions.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.SeansSpace.RevExtensions.ExpansionHubMotor;

/**
 * Created by Sean Cardosi on 2019-12-01.
 */
@Disabled

@TeleOp(name = "Hub Current Monitor",group = "Diagnostics")
public class CurrentMonitor extends OpMode
{
    ExpansionHubMotor lf, lr, rf, rr;
    ExpansionHubEx expansionHub;

    @Override
    public void init() {
        /*
         * Before init() was called on this user code, REV Extensions 2
         * was notified via OpModeManagerNotifier.Notifications and
         * it automatically took care of initializing the new objects
         * in the hardwaremap for you. Historically, you would have
         * needed to call RevExtensions2.init()
         */
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        lf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftB");
        lr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftF");
        rf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightB");
        rr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightF");
    }

    @Override
    public void loop()
    {
        /*
         * ------------------------------------------------------------------------------------------------
         * Current monitors - NOTE: units are amps
         * ------------------------------------------------------------------------------------------------
         */

        String header =
                "**********************************\n" +
                "        CURRENT MONITORING        \n" +
                "**********************************\n";
        telemetry.addLine(header);

        telemetry.addData("Total Current", expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");
        telemetry.addData("I2C Current", expansionHub.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");
        telemetry.addData("General Purpous Input/Output Current", expansionHub.getGpioBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");
        telemetry.addData("Left Front Current", lf.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");
        telemetry.addData("Left Back Current", lr.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");
        telemetry.addData("Right Front Current", rf.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");
        telemetry.addData("Right Back Current", rr.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "Amps");

        telemetry.update();
    }
}