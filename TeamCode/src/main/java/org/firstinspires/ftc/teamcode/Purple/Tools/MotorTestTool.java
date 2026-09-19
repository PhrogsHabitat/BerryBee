package org.firstinspires.ftc.teamcode.Purple.Tools;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Panels;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Names;

@TeleOp(name = "Motor Test Tool", group = "Purple")
public class MotorTestTool extends OpMode
{
    private Controls driver1;
    private static final double TEST_POWER = 0.25;

    private final MotorConfig[] motors = new MotorConfig[4];

    private TelemetryManager panelsTelemetry;
    private int selectedMotorIndex = 0;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private double swag = 0.0;

    @Override
    public void init ()
    {
        panelsTelemetry = Panels.telemetry();

        motors[0] = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 112, 6000)
                .disableVelocityControl().build();
        motors[1] = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 112, 6000)
                .disableVelocityControl().build();
        motors[2] = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 415.2, 1620)
                .disableVelocityControl().build();
        motors[3] = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 415.2, 1620)
                .disableVelocityControl().build();

        driver1 = new Controls(gamepad1);
    }

    @Override
    public void loop ()
    {
        driver1.update();

        motors[0].setPower(swag);
        motors[1].setPower(swag);
        motors[2].setPower(swag);
        motors[3].setPower(swag);

        if (driver1.justPressed("dpad_up"))
        {
            swag += 0.1;
        }
        else if (driver1.justPressed("dpad_down"))
        {
            swag -= 0.1;
        }
    }

    @Override
    public void stop ()
    {
        stopAll();
    }

    private void handleSelection ()
    {

    }

    private void stopAll ()
    {
        for (MotorConfig motor : motors)
        {
            if (motor != null)
            {
                motor.setPower(0.0);
            }
        }
    }
}
