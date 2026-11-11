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
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "BasicAuto", group = "Purple")
public class BasicAuto extends PurpleOpMode {

    private Follower follower;

    public Explosher explosher;
    public Vaccum vaccum;

    public ElapsedTime flickTimer;

    private final ElapsedTime saveTimer = new ElapsedTime();
    private static final long SAVE_INTERVAL_MS = 1000;

    public boolean shouldExplode = false;
    public boolean shouldFlick = false;
    public boolean shouldSuck = false;

    private final Pose startPose = new Pose(56, 10, Math.toRadians(270));
    private final Pose startPose2 = new Pose(50, 19, Math.toRadians(245));
    private final Pose nectarPose1 = new Pose(10, 86, Math.toRadians(90));
    private final Pose nectarPose2 = new Pose(10, 95, Math.toRadians(90));
    private final Pose shootPose = new Pose(39, 125, Math.toRadians(128));
    private final Pose shootPose2 = new Pose(32, 122, Math.toRadians(120));

    private final Pose pollenPose = new Pose(10, 10, Math.toRadians(270));
    private final Pose homePose = new Pose(14, 90, Math.toRadians(180));
    private final Pose curvePose = new Pose(30, 105);
    private PurplePathing pathManager;

    @Override
    public void create() {

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(new Pose(56, 10, Math.toRadians(270)));

        PurpleMemory.initialize(hardwareMap, follower);

        pathManager = new PurplePathing(follower);

        explosher = new Explosher(hardwareMap);
        vaccum = new Vaccum(hardwareMap);

        explosher.setGateState(Explosher.FLICKER, Explosher.GateState.PASS);
        explosher.setGateState(Explosher.BLOCKER, Explosher.GateState.PASS);

        flickTimer = new ElapsedTime();
        flickTimer.startTime();

        saveTimer.reset();

        // Note: durationSec removed; only waitTimeSec remains.
        PurplePath path1 = new PurplePath("stay still and shoot",
                Paths.line(startPose, startPose2).linear(startPose, startPose2), 4)
                .onStart(() -> path1OnStart())
                .onComplete(() -> path1Oncomplete());

        PurplePath path2 = new PurplePath("Get TO nector",
                Paths.line(startPose2, nectarPose1).linear(startPose2, nectarPose1), 0.5)
                .onStart(() -> path2OnStart())
                .onComplete(() -> path2Oncomplete());

        PurplePath path3 = new PurplePath("get THE nector",
                Paths.line(nectarPose1, nectarPose2).linear(nectarPose1, nectarPose2), 0.5)
                .onStart(() -> path3OnStart())
                .onComplete(() -> path3Oncomplete());

        PurplePath path4 = new PurplePath("move to shoot",
                Paths.line(nectarPose2, shootPose).linear(nectarPose2, shootPose), 4)
                .onStart(() -> path4OnStart())
                .onComplete(() -> path4Oncomplete());

        PurplePath path5 = new PurplePath("grab pollen",
                Paths.curve(shootPose, curvePose, pollenPose).linear(shootPose, pollenPose), 0.5)
                .onStart(() -> path5OnStart())
                .onComplete(() -> path5Oncomplete());

        PurplePath path6 = new PurplePath("move to shoot again",
                Paths.curve(pollenPose, curvePose, shootPose2).linear(pollenPose, shootPose2), 4)
                .onStart(() -> path6OnStart())
                .onComplete(() -> path6Oncomplete());

        PurplePath path7 = new PurplePath("go home",
                Paths.line(shootPose, homePose).linear(shootPose, homePose), 1)
                .onStart(() -> path7OnStart())
                .onComplete(() -> path7Oncomplete());

        PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6, path7)
                .onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

        pathManager.startChain(chain, true, () -> savePose());

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
        // Intake on
        shouldSuck = true;
    }
    public void path3Oncomplete()
    {
        // Intake off
        shouldSuck = false;
    }

    public void path4OnStart()
    {
        shouldExplode = true;
    }
    public void path4Oncomplete()
    {
        // Explo off
        shouldFlick = true;
    }

    public void path5OnStart()
    {
        // Intake on
        shouldSuck = true;
        shouldExplode = false;
        shouldFlick = false;
    }
    public void path5Oncomplete()
    {

    }

    public void path6OnStart()
    {
        // intake off
        shouldSuck = false;
        shouldExplode = true;
    }
    public void path6Oncomplete()
    {
        // explo on
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
        savePose();
    }

    @Override
    public void update() {

        pathManager.update();
        explosher.update();
        vaccum.update();

        updateExplosher();
        updateFingers();
        updateVaccum();

        // Periodic save so a force-stop mid-auto still leaves a usable pose.
        if (saveTimer.milliseconds() > SAVE_INTERVAL_MS)
        {
            savePose();
            saveTimer.reset();
        }

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

    private void savePose()
    {
        if (follower == null || PurpleMemory.Instance == null
                || PurpleMemory.Instance.persist() == null)
        {
            return;
        }

        Pose finalPose = follower.pose();
        if (finalPose == null)
        {
            return;
        }

        // Batched so X/Y/Heading are written in a single atomic transaction.
        Map<String, String> values = new HashMap<>();
        values.put("PrevX", String.valueOf(finalPose.x()));
        values.put("PrevY", String.valueOf(finalPose.y()));
        values.put("PrevHeading", String.valueOf(Math.toDegrees(finalPose.heading())));
        PurpleMemory.Instance.persist().setAll(values);
    }

    @Override
    public void destroy()
    {
        savePose();
    }

}