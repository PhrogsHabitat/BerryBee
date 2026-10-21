package org.firstinspires.ftc.teamcode.Purple.Auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;

@Autonomous(name = "BackBluePedroAuto", group = "Purple")
public class BackBluePedroAuto extends PurpleOpMode {

    private Follower follower;

    public Explosher explosher;
    public Vaccum vaccum;

    public ElapsedTime shootTimer;

    private final Pose startPose = new Pose(62.5, 10, Math.toRadians(90));

    private PurplePathing pathManager;

    @Override
    public void create()
    {

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,0));
        pathManager = new PurplePathing(follower);

        shootTimer = new ElapsedTime();

        explosher = new Explosher(hardwareMap);
        vaccum = new Vaccum(hardwareMap);


        PurplePath path1 = new PurplePath("Swag", line(follower.pose(), startPose).linear(follower.pose(), startPose), 5, 5);

        PurpleChain chain = new PurpleChain(path1)
                .onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

        pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));

        shootTimer.reset();

        DebugUtil.setTelemetry(telemetry);
    }

    @Override
    public void update()
    {
        // update the manager (must be called every loop)
        pathManager.update();
        explosher.update();
        vaccum.update();

        // logging
        DebugUtil.logAdd("Current Path: " + pathManager.curPath());
        DebugUtil.update();

    }

    @Override
    public void destroy()
    {

    }
}