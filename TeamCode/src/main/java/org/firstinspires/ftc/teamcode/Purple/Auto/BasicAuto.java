package org.firstinspires.ftc.teamcode.Purple.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@Autonomous(name = "BasicAuto", group = "Purple")
public class BasicAuto extends PurpleOpMode {

    private Follower follower;

    public Explosher explosher;
    public Vaccum vaccum;

    public ElapsedTime flickTimer;

    public boolean shouldExplode = false;
    public boolean shouldFlick = false;
    public boolean shouldSuck = false;


    private final Pose startPose = new Pose(56, 10, Math.toRadians(270));
    private final Pose startPose2 = new Pose(35.5, 19, Math.toRadians(230));
    private final Pose nectarPose1 = new Pose(10, 86, Math.toRadians(90));
    private final Pose nectarPose2 = new Pose(10, 100, Math.toRadians(90));
    private final Pose shootPose = new Pose(54, 118, Math.toRadians(90));
    private final Pose pollenPose = new Pose(10, 10, Math.toRadians(270));
    private final Pose homePose = new Pose(10, 90, Math.toRadians(180));
    private final Pose curvePose = new Pose(30, 105);

    private PurplePathing pathManager;

    @Override
    public void create() {

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(new Pose(56,10, Math.toRadians(270)));

        pathManager = new PurplePathing(follower);

        explosher = new Explosher(hardwareMap);
        vaccum = new Vaccum(hardwareMap);

        explosher.setGateState(Explosher.FLICKER, Explosher.GateState.PASS);
        explosher.setGateState(Explosher.BLOCKER, Explosher.GateState.PASS);

        flickTimer = new ElapsedTime();
        flickTimer.startTime();

        PurplePath path1 = new PurplePath("stay still and shoot", Paths.line(startPose, startPose2).linear(startPose, startPose2), 3, 4)
                .onStart(() -> path1OnStart())
                .onComplete(() -> path1Oncomplete());

        PurplePath path2 = new PurplePath("Get TO nector", Paths.line(startPose2, nectarPose1).linear(startPose2, nectarPose1), 3, 0.5)
                .onStart(() -> path2OnStart())
                .onComplete(() -> path2Oncomplete());

        PurplePath path3 = new PurplePath("get THE nector", Paths.line(nectarPose1, nectarPose2).linear(nectarPose1, nectarPose2), 1, 0.5)
                .onStart(() -> path3OnStart())
                .onComplete(() -> path3Oncomplete());

        PurplePath path4 = new PurplePath("move to shoot", Paths.line(nectarPose2, shootPose).linear(nectarPose2, shootPose), 1, 4)
                .onStart(() -> path4OnStart())
                .onComplete(() -> path4Oncomplete());

        PurplePath path5 = new PurplePath("grab pollen", Paths.curve(shootPose, curvePose, pollenPose).linear(shootPose, pollenPose), 5, 0.5)
                .onStart(() -> path5OnStart())
                .onComplete(() -> path5Oncomplete());

        PurplePath path6 = new PurplePath("shoot again", Paths.curve(pollenPose, curvePose, shootPose).linear(pollenPose, shootPose), 4, 4)
                .onStart(() -> path6OnStart())
                .onComplete(() -> path6Oncomplete());

        PurplePath path7 = new PurplePath("go home", Paths.line(shootPose, homePose).linear(shootPose, homePose), 1, 0)
                .onStart(() -> path7OnStart())
                .onComplete(() -> path7Oncomplete());

        PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6, path7)
                .onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

        pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));

        DebugUtil.setTelemetry(telemetry);
    }


    public void path1OnStart()
    {
        // Explo on
        flickTimer.reset();
        shouldExplode = true;
    }
    public void path1Oncomplete()
    {
        shouldFlick = true;
    }

    public void path2OnStart()
    {
        // Explo off
        // Intake off
        shouldExplode = false;
        shouldFlick = false;

    }
    public void path2Oncomplete()
    {

    }

    public void path3OnStart()
    {
        // intake on
        shouldSuck = true;
    }
    public void path3Oncomplete()
    {
        // intake off
        shouldSuck = false;
    }

    public void path4OnStart()
    {
        //
    }
    public void path4Oncomplete()
    {
        // explo off
        shouldExplode = true;
        shouldFlick = true;
    }

    public void path5OnStart()
    {
        // intake on
        shouldSuck = true;
        shouldExplode = false;
        shouldFlick = false;
    }
    public void path5Oncomplete()
    {
        // intake off
        shouldSuck = false;
    }

    public void path6OnStart()
    {

    }
    public void path6Oncomplete()
    {
        // explo on
        shouldExplode = true;
        shouldFlick = true;
    }

    public void path7OnStart()
    {
        // explo off
        shouldExplode = false;
        shouldFlick = false;
    }
    public void path7Oncomplete()
    {

    }

    @Override
    public void update() {

        pathManager.update();
        explosher.update();
        vaccum.update();

        updateExplosher();
        updateFingers();
        updateVaccum();

        // logging
        DebugUtil.logAdd("Current Path: " + pathManager.curPath());
        DebugUtil.update();
    }

    public void updateExplosher()
    {
        if (shouldExplode)
        {
            exploToggle(true);
        }
        else
        {
            exploToggle(false);
        }
    }

    public void updateFingers()
    {
        if (shouldFlick)
        {
            gateToggle(Explosher.FLICKER, false);
            if (flickTimer.milliseconds() > 1000) {
                gateToggle(Explosher.FLICKER, true);
                flickTimer.reset();
                flickTimer.startTime();
            }
            gateToggle(Explosher.BLOCKER, false);
        }
        else
        {
            // Reset timers or whatever
            gateToggle(Explosher.FLICKER, true);
            gateToggle(Explosher.BLOCKER, true);
        }
    }

    public void updateVaccum()
    {
        if (shouldSuck)
        {
            suckToggle(true);
        }
        else
        {
            suckToggle(false);
        }
    }

    public void gateToggle(int gate, boolean open)
    {
        Explosher.GateState block = open ? Explosher.GateState.PASS : Explosher.GateState.STOP;

        explosher.setGateState(gate, block);
    }

    public void exploToggle(boolean on)
    {
        int motor1 = on ? 2900 : 0;
        int motor2 = on ? 2900 : 0;

        explosher.setRPM(1, motor1);
        explosher.setRPM(2, motor2);
    }

    private void suckToggle(boolean suck) {
        if (suck) {
            vaccum.setPower(1.0);
        }
        else {
            vaccum.setPower(0);
        }
    }

    @Override
    public void destroy()
    {

    }

}
