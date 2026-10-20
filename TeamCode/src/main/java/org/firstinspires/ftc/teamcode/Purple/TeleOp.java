
package org.firstinspires.ftc.teamcode.Purple;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import static com.pedropathing.api.Paths.line;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
		name = "PurpleTeleOp",
		group = "Purple"
)
@Configurable
public class TeleOp extends PurpleOpMode
{
	private static final Pose DEFAULT_TELEOP_START_POSE =
			new Pose(0, 0, Math.toRadians(90));

	public static Pose startingPose;

	// Pollen: 2900 RPM
	// Nectar: 3300 RPM
	public double swagRPM = 2900;
	public double swagRPM2 = 3300;

	private Controls driver1;
	private Controls driver2;

	private Follower follower;
	private Vaccum vaccum;
	private Explosher explosher;

	public static double SNAP_RANGE = 25.0;

	public static Pose[] SNAP_POINTS =
			{
					new Pose(0, 0, Math.toRadians(90)),
					new Pose(10, 80, Math.toRadians(90)),
					new Pose(72, 72, Math.toRadians(0))
			};

	private boolean snapActive = false;
	private Pose activeSnapPoint = null;

	@Override
	public void create()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedro.Constants
				.create(hardwareMap);

		follower.setPose(DEFAULT_TELEOP_START_POSE);
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
		if (follower != null)
		{
			follower.stop();
		}

		if (vaccum != null)
		{
			vaccum.stop();
		}

		if (explosher != null)
		{
			explosher.stop();
		}
	}

	private void updateDrive()
	{
		boolean snapButtonDown = driver1.isPressed("right_bumper");

		// Start the path once per button press.
		if (driver1.justPressed("right_bumper"))
		{
			Pose currentPose = follower.pose();

			Pose targetPose = new Pose(0, 0, Math.toRadians(0));

			Path snapPath = line(currentPose, targetPose)
					.linear(currentPose, targetPose);

			follower.follow(snapPath);

			activeSnapPoint = targetPose;
			snapActive = true;
		}

		if (snapButtonDown)
		{

			return;
		}

		if (snapActive)
		{
			follower.stop();

			activeSnapPoint = null;
			snapActive = false;
		}

		double forward = -gamepad1.left_stick_y
				* Constants.DRIVE_POWER_SCALE;

		double strafe = -gamepad1.left_stick_x
				* Constants.DRIVE_POWER_SCALE;

		double turn = -gamepad1.right_stick_x
				* Constants.DRIVE_POWER_SCALE;

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

		if (driver2.getRightStickY() > 0.5)
		{
			explosher.setRPM(1, swagRPM);
			explosher.setRPM(2, swagRPM2);
		}
		else if (driver2.getRightStickY() < -0.5)
		{
			explosher.setRPM(1, 0);
			explosher.setRPM(2, 0);
		}

		if (driver2.isPressed("right_trigger"))
		{
			explosher.setGateState(Explosher.GateState.PASS);
		}
		else
		{
			explosher.setGateState(Explosher.GateState.STOP);
		}

		// Adjust pollen RPM.
		if (driver1.justPressed("dpad_up"))
		{
			swagRPM += 100;
		}

		if (driver1.justPressed("dpad_down"))
		{
			swagRPM -= 100;
		}

		// Adjust nectar RPM.
		if (driver1.justPressed("dpad_right"))
		{
			swagRPM2 += 100;
		}

		if (driver1.justPressed("dpad_left"))
		{
			swagRPM2 -= 100;
		}
	}

	private void updateTelemetry()
	{
		Pose pose = follower.pose();

		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose);
		DebugUtil.logAdd("[SNAP] ACTIVE: " + snapActive);
		DebugUtil.logAdd("[SNAP] TARGET: " + activeSnapPoint);
		DebugUtil.logAdd("[SNAP] RANGE: " + SNAP_RANGE);
		DebugUtil.logAdd("[SNAP] FOLLOWING: " + follower.following());
	}
}
