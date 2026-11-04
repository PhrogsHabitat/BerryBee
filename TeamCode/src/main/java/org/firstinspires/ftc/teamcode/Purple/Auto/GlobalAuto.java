package org.firstinspires.ftc.teamcode.Purple.Auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "GlobalAuto", group = "Autonomous")
public class GlobalAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9.5, 90);
    private final Pose point1Start = poseFactory.of(56, 9.5, 270);
    private final Pose point1 = poseFactory.of(66, 9.5, 270);
    private final Pose path1 = poseFactory.of(12, 91, 0);
    private final Pose path1Control1 = poseFactory.of(19.3069, 46.885, 0);

    public Explosher explosher;

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
        PurpleMemory.Instance.persist().set("PrevX", Double.toString(follower.pose().x()));
        PurpleMemory.Instance.persist().set("PrevY", Double.toString(follower.pose().y()));
        PurpleMemory.Instance.persist().set("PrevHeading", Double.toString(follower.pose().heading()));
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
                gateClosed,
                exploOn,
                follow(follower, path1()),
                gateOpen,
                waitMs(5000),
                gateClosed,
                exploOff,
                follow(follower, path1_2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        explosher = new Explosher(hardwareMap);
        explosher.setRPM(1, 0);
        explosher.setRPM(2, 0);
        explosher.setGateState(Explosher.BLOCKER, Explosher.GateState.STOP);
        explosher.setGateState(Explosher.FLICKER, Explosher.GateState.STOP);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            explosher.update();
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
        return line(point1Start, point1).linear(point1Start, point1);
    }

    public Path path1_2() {
        return curve(point1, path1Control1, path1).linear(point1, path1);
    }
}
