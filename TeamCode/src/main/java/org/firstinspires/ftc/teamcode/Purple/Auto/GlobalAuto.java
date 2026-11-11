package org.firstinspires.ftc.teamcode.Purple.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@Autonomous(name = "GlobalAuto", group = "Purple")
public class GlobalAuto extends PurpleOpMode {

    public ElapsedTime swagTimer;
    public boolean started = false;

    public MotorConfig fr;
    public MotorConfig fl;
    public MotorConfig br;
    public MotorConfig bl;

    @Override
    public void create()
    {
        // Motor stuff
        fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                .build();

        fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                .inverted()
                .build();

        br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                .build();

        bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                .inverted()
                .build();

        swagTimer = new ElapsedTime();
        swagTimer.reset();
    }


    @Override
    public void update()
    {
        if (!started)
        {
            swagTimer.reset();
            swagTimer.startTime();

            started = true;
        }

        if (swagTimer.milliseconds() > 12000)
        {
            fr.setPower(0.3);
            fl.setPower(0.3);
            br.setPower(0.3);
            bl.setPower(0.3);
        }
        else
        {
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    @Override
    public void destroy()
    {

    }

}
