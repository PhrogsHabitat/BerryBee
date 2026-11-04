//package org.firstinspires.ftc.teamcode.Purple.Auto;
//
//import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.api.Paths.path;
//
//import com.pedropathing.api.Paths;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
//import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
//import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
//import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
//import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
//import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
//
//@Autonomous(name = "RedAuto", group = "Purple")
//public class RedAuto extends OpMode {
//
//    private Follower follower;
//
//    public Explosher explosher;
//    public Vaccum vaccum;
//
//    public ElapsedTime shootTimer;
//
//    private final Pose startPose = new Pose(56, 10, Math.toRadians(90));
//    private final Pose nectarPose1 = new Pose(10, 86, Math.toRadians(90));
//    private final Pose nectarPose2 = new Pose(10, 100, Math.toRadians(90));
//    private final Pose shootPose = new Pose(56.5, 118, Math.toRadians(270));
//    private final Pose pollenPose = new Pose(9, 9, Math.toRadians(270));
//    private final Pose homePose = new Pose(9.5, 90, Math.toRadians(180));
//    private final Pose curvePose = new Pose(25, 118);
//
//    private PurplePathing pathManager;
//
//    @Override
//    public void init() {
//
//        onCreate();
//    }
//
//    @Override
//    public void loop() {
//
//        onUpdate();
//    }
//
//    private void onCreate() {
//
//        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
////        follower.setPose(new Pose(56,10,90));
//        follower.setPose(new Pose(9.5,34.5,0));
//
//        pathManager = new PurplePathing(follower);
//
//        shootTimer = new ElapsedTime();
//
//        explosher = new Explosher(hardwareMap);
//        vaccum = new Vaccum(hardwareMap);
//
//        PurplePath path1 = new PurplePath("Get TO nector", Paths.line(startPose, nectarPose1).linear(startPose, nectarPose1), 3, 2);
//
//        PurplePath path2 = new PurplePath("get THE nector", Paths.line(nectarPose1, nectarPose2).linear(nectarPose1, nectarPose2), 1, 2);
//
//        PurplePath path3 = new PurplePath("move to shoot", Paths.line(nectarPose2, shootPose).linear(nectarPose2, shootPose), 1.5, 2);
//
//        PurplePath path4 = new PurplePath("grab pollen", Paths.curve(shootPose, curvePose, pollenPose).linear(shootPose, pollenPose), 3, 2);
//
//        PurplePath path5 = new PurplePath("shoot again", Paths.curve(pollenPose, curvePose, shootPose).linear(pollenPose, shootPose), 3, 2);
//
//        PurplePath path6 = new PurplePath("go home", Paths.line(shootPose, homePose).linear(shootPose, homePose), 1, 2);
//
//
//        PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6)
//                .onComplete(() -> DebugUtil.logAdd("Chain fully finished"));
//
//        pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));
//
//        shootTimer.reset();
//
//        DebugUtil.setTelemetry(telemetry);
//    }
//
//    private void onUpdate() {
//        // update the manager (must be called every loop)
//        pathManager.update();
//        explosher.update();
//        vaccum.update();
//
//        // logging
//        DebugUtil.logAdd("Current Path: " + pathManager.curPath());
//        DebugUtil.update();
//    }
//
//}


package org.firstinspires.ftc.teamcode.Purple.Auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class RedAuto extends LinearOpMode {

    private Follower follower;
    public Explosher explosher;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9.5, 34.5, 90);
    private final Pose path1Start = poseFactory.of(9.5, 34.5, 0);
    private final Pose path1 = poseFactory.of(29, 16, 230);
    private final Pose point2 = poseFactory.of(12.6696, 93.9453, 90);

    Command exploOn = instant(() -> exploToggle(true));
    Command exploOff = instant(() -> exploToggle(false));
    Command gateOpen = instant(() -> gateToggle(true));
    Command gateClosed = instant(() -> gateToggle(false));

    public void gateToggle(boolean open)
    {
        Explosher.GateState block = open ? Explosher.GateState.PASS : Explosher.GateState.STOP;

        explosher.setGateState(Explosher.BLOCKER, block);
    }

    public void savePose()
    {
        PurpleMemory.Instance.persist().set("PrevPose", follower.pose().toString());
    }

    public void exploToggle(boolean on)
    {
        int motor1 = on ? 2900 : 0;
        int motor2 = on ? 3100 : 0;

        explosher.setRPM(1, motor1);
        explosher.setRPM(2, motor2);
    }

    public Command autoRoutine() {
        return sequential(
                exploOn,
                follow(follower, path1()),
                waitMs(1000),
                gateOpen,
                waitMs(5000),
                gateClosed,
                exploOff,
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        explosher = new Explosher(hardwareMap);
        explosher.setRegressionEnabled(false);

        PurpleMemory.initialize(hardwareMap, follower);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            explosher.update();
            PurpleMemory.Instance.update();

            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }
}
