package org.firstinspires.ftc.teamcode.Purple;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
@Configurable
public class TeleOp extends PurpleOpMode
{
	private static final Pose DEFAULT_TELEOP_START_POSE = new Pose(72, 72, Math.toRadians(0.0));
	public static Pose startingPose;
	// Pollen 3000
	// Necter 3300

	public double swagRPM = 2500;

	private Controls driver1;
	private Controls driver2;

	private Follower follower;
	private Vaccum vaccum;
	private Explosher explosher;

	@Override
	public void create()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		// PedroPathing 3.0 Follower creation.
		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);

		// Pedro 3 uses setPose() to initialize the localizer pose.
		follower.setPose(startingPose == null ? DEFAULT_TELEOP_START_POSE : startingPose);
		follower.update();

		PurpleMemory.initialize(hardwareMap, follower);

		vaccum = new Vaccum(hardwareMap);
		explosher = new Explosher(hardwareMap);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update()
	{
		driver1.update();
		driver2.update();

		// Pedro must be updated every loop so localization and drivetrain control run.
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
	public void destroy()
	{
		vaccum.stop();
		explosher.stop();
	}

	private void updateDrive()
	{
		double forward = -gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE;
		double strafe = -gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE;
		double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		// PedroPathing 3.0 manual TeleOp API.
		follower.manual(forward, strafe, turn);
	}

	private void updateVaccum()
	{
		if (driver2.isPressed("x"))
		{
			vaccum.setPower(1.0);
		}
		else if (driver2.isPressed("y"))
		{
			vaccum.setPower(-1.0);
		}
		else
		{
			vaccum.setPower(0.0);
		}
	}

	private void updateExplosher()
	{
		explosher.setRegressionEnabled(false);
		explosher.setRPM(swagRPM);

		if (driver1.justPressed("dpad_up"))
		{
			swagRPM += 100;
		}
		if (driver1.justPressed("dpad_down"))
		{
			swagRPM -= 100;
		}
	}

	private void updateTelemetry()
	{
		Pose pose = follower.pose();

		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose);
	}
}
