package org.firstinspires.ftc.teamcode.Purple.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@Autonomous(name = "RedAuto", group = "Purple")
public class RedAuto extends OpMode {

    private Follower follower;

    public Explosher explosher;
    public Vaccum vaccum;

    public ElapsedTime shootTimer;

    private final Pose startPose = new Pose(56, 10, Math.toRadians(90));
    private final Pose nectarPose1 = new Pose(10, 86, Math.toRadians(90));
    private final Pose nectarPose2 = new Pose(10, 100, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.5, 118, Math.toRadians(270));
    private final Pose pollenPose = new Pose(9, 9, Math.toRadians(270));
    private final Pose homePose = new Pose(9.5, 90, Math.toRadians(180));
    private final Pose curvePose = new Pose(25, 118);

    private PurplePathing pathManager;

    @Override
    public void init() {

        onCreate();
    }

    @Override
    public void loop() {

        onUpdate();
    }

    private void onCreate() {

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(new Pose(56,10,90));
        pathManager = new PurplePathing(follower);

        shootTimer = new ElapsedTime();

        explosher = new Explosher(hardwareMap);
        vaccum = new Vaccum(hardwareMap);

        PurplePath path1 = new PurplePath("Get TO nector", Paths.line(startPose, nectarPose1).linear(startPose, nectarPose1), 2, 0);

        PurplePath path2 = new PurplePath("get THE nector", Paths.line(nectarPose1, nectarPose2).linear(nectarPose1, nectarPose2), 1, 0);

        PurplePath path3 = new PurplePath("move to shoot", Paths.line(nectarPose2, shootPose).linear(nectarPose2, shootPose), 1.5, 0);

        PurplePath path4 = new PurplePath("grab pollen", Paths.curve(shootPose, curvePose, pollenPose).linear(shootPose, pollenPose), 3, 0);

        PurplePath path5 = new PurplePath("shoot again", Paths.curve(pollenPose, curvePose, shootPose).linear(pollenPose, shootPose), 3, 0);

        PurplePath path6 = new PurplePath("go home", Paths.line(shootPose, homePose).linear(shootPose, homePose), 1, 0);


        PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6)
                .onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

        pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));

        shootTimer.reset();

        DebugUtil.setTelemetry(telemetry);
    }

    private void onUpdate() {
        // update the manager (must be called every loop)
        pathManager.update();
        explosher.update();
        vaccum.update();

        // logging
        DebugUtil.logAdd("Current Path: " + pathManager.curPath());
        DebugUtil.update();
    }

}
