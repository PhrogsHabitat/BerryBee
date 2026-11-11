package org.firstinspires.ftc.teamcode.Purple.Components.Lime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Target Test", group = "PurpleTests")
public class SourOp extends OpMode
{
    @Override
    public void init()
    {
        boolean success = LimeUtil.start(hardwareMap, 100);
        LimeUtil.setPipeline(1);

        telemetry.addData("Status", success ? "Initialized" : "Init Failed");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        LimeUtil.update();

        boolean hasTarget = LimeUtil.hasValidTarget();
        telemetry.addData("Target Valid", hasTarget);

        if (hasTarget)
        {
            telemetry.addData("TX", LimeUtil.getTx());
            telemetry.addData("TY", LimeUtil.getTy());
            telemetry.addData("TA", LimeUtil.getTa());
        }

        telemetry.update();
    }

    @Override
    public void stop()
    {
        LimeUtil.stop();
    }
}



