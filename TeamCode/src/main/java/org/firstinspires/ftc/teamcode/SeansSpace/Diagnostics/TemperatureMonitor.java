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

@TeleOp(name = "Temperature Monitor",group = "Diagnostics")
public class TemperatureMonitor extends OpMode
{
    ExpansionHubMotor lf, lr, rf, rr;
    ExpansionHubEx expansionHub;

    @Override
    public void init() {
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        lf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftB");
        lr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftF");
        rf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightB");
        rr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightF");
    }

    @Override
    public void loop() {
        /*
         * ------------------------------------------------------------------------------------------------
         * Temperature monitors
         * ------------------------------------------------------------------------------------------------
         */

        String header =
                "**********************************\n" +
                "       TEMPERATURE MONITORS       \n" +
                "**********************************\n";
        telemetry.addLine(header);

        telemetry.addData("Module Temperature", expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "F");
        telemetry.addData("Module Over Temp", expansionHub.isModuleOverTemp());
        telemetry.addData("Left Front H-bridge Over Temp", lf.isBridgeOverTemp());
        telemetry.addData("Left Back H-bridge Over Temp", lr.isBridgeOverTemp());
        telemetry.addData("Right Front H-bridge Over Temp", rf.isBridgeOverTemp());
        telemetry.addData("Right Back H-bridge Over Temp", rr.isBridgeOverTemp());

        telemetry.update();
    }
}