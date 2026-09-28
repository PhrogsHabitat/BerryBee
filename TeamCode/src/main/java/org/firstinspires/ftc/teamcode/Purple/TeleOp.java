package org.firstinspires.ftc.teamcode.Purple;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.AutoRotateController;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
@Configurable
public class TeleOp extends PurpleOpMode
{
	private static final Pose DEFAULT_TELEOP_START_POSE = new Pose(72, 72, Math.toRadians(0.0));

	public static Pose startingPose;

	private Controls driver1;
	private Controls driver2;
	private Follower follower;

	private Vaccum vaccum;
	private Explosher explosher;


	@Override
	public void create ()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setStartingPose(startingPose == null ? DEFAULT_TELEOP_START_POSE : startingPose);
		follower.update();
		follower.startTeleopDrive(true);

		PurpleMemory.initialize(hardwareMap, follower);
		//  LimeUtil.start(hardwareMap, 60);
		// LimeUtil.setPipeline(0);

		vaccum = new Vaccum(hardwareMap);
		explosher = new Explosher(hardwareMap);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{
		driver1.update();
		driver2.update();
		follower.update();
		PurpleMemory.Instance.update();

		vaccum.update();
		explosher.update();

		updateDrive();
		updateVaccum();
		updateExplosher();

		updateTelemetry();
		DebugUtil.update();
	}

	@Override
	public void destroy ()
	{
		// explosher.stop();
		   vaccum.stop();
		   explosher.stop();
	}

	private void updateDrive ()
	{
		double forward = -gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE;
		double strafe = -gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE;
		double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		follower.setTeleOpDrive(forward, strafe, turn, false);
	}

	private void updateVaccum()
	{
		// Basic controls for the vaccum, X intake, Y outtake
		if (driver2.isPressed("x"))
		{
			vaccum.setPower(1.0);
		}
		else if (driver2.isPressed("y"))
		{
			vaccum.setPower(-1.0);
		}
	}

	private void updateExplosher()
	{
		explosher.setRegressionEnabled(false);

		explosher.setRPM(4500);
	}

	private void updateTelemetry ()
	{
		Pose pose = follower.getPose();

		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose);
	}
}
