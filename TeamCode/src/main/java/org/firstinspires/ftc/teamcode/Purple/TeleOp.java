package org.firstinspires.ftc.teamcode.Purple;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

import static com.pedropathing.api.Paths.line;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
		name = "PurpleTeleOp",
		group = "Purple"
)
@Configurable
public class TeleOp extends PurpleOpMode
{
	private static final Pose DEFAULT_TELEOP_START_POSE =
			new Pose(0, 0, Math.toRadians(0));

	public static Pose startingPose;

	public static boolean DEBUG_MODE = true;
	public static double DEBUG_RPM_STEP = 100.0;
	public static double DEBUG_FINGER_STEP = 0.02;

	// Pollen: 2900 RPM
	// Nectar: 3300 RPM
	public double swagRPM = 2900;
	public double swagRPM2 = 3300;

	public double debugBlockerPos = 0.5;
	public double debugFlickerPos = 0.5;

	private Controls driver1;
	private Controls driver2;

	private Follower follower;
	private Vaccum vaccum;
	private Explosher explosher;

	public static double SNAP_RANGE = 25.0;

	public static Pose[] SNAP_POINTS =
			{
					new Pose(0, 0, Math.toRadians(0)),
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

		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);

		follower.setPose(startingPose == null ? DEFAULT_TELEOP_START_POSE : startingPose);
		follower.update();

		// PurpleMemory.initialize(hardwareMap, follower);

		vaccum = new Vaccum(hardwareMap);
		explosher = new Explosher(hardwareMap);

		if (DEBUG_MODE)
		{
			debugBlockerPos = explosher.getPosition(Explosher.BLOCKER);
			debugFlickerPos = explosher.getPosition(Explosher.FLICKER);
		}

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update()
	{
		driver1.update();
		driver2.update();

		follower.update();

		// PurpleMemory.Instance.update();

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

		if (driver1.justPressed("start"))
		{
			follower.setPose(new Pose(0, 0, 90));
		}

		if (driver1.justPressed("right_bumper"))
		{
			Pose currentPose = follower.pose();

			Pose targetPose = new Pose(0, 0, 90);

			Path snapPath = line(currentPose, targetPose).linear(currentPose, targetPose);

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

		double forward = -gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE;
		double strafe = -gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE;
		double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		ManualDrive.driveOrHold(follower, forward, strafe, turn);
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

		if (DEBUG_MODE)
		{
			updateExplosherDebug();
			return;
		}

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
			explosher.setGateState(Explosher.BLOCKER, Explosher.GateState.STOP);
			explosher.setGateState(Explosher.FLICKER, Explosher.GateState.STOP);
		}
		else
		{
			explosher.setGateState(Explosher.BLOCKER, Explosher.GateState.PASS);
			explosher.setGateState(Explosher.FLICKER, Explosher.GateState.STOP);
		}
	}

	private void updateExplosherDebug()
	{
		if (driver1.justPressed("dpad_up"))
		{
			swagRPM += DEBUG_RPM_STEP;
		}
		if (driver1.justPressed("dpad_down"))
		{
			swagRPM -= DEBUG_RPM_STEP;
		}
		if (driver1.justPressed("dpad_right"))
		{
			swagRPM2 += DEBUG_RPM_STEP;
		}
		if (driver1.justPressed("dpad_left"))
		{
			swagRPM2 -= DEBUG_RPM_STEP;
		}

		swagRPM = MathUtil.clamp(swagRPM, 0.0, explosher.getMaxRPM());
		swagRPM2 = MathUtil.clamp(swagRPM2, 0.0, explosher.getMaxRPM());

		explosher.setRPM(1, swagRPM);
		explosher.setRPM(2, swagRPM2);

		if (driver2.justPressed("dpad_up"))
		{
			debugBlockerPos += DEBUG_FINGER_STEP;
		}
		if (driver2.justPressed("dpad_down"))
		{
			debugBlockerPos -= DEBUG_FINGER_STEP;
		}
		if (driver2.justPressed("dpad_right"))
		{
			debugFlickerPos += DEBUG_FINGER_STEP;
		}
		if (driver2.justPressed("dpad_left"))
		{
			debugFlickerPos -= DEBUG_FINGER_STEP;
		}

		debugBlockerPos = MathUtil.clamp(debugBlockerPos, 0.0, 1.0);
		debugFlickerPos = MathUtil.clamp(debugFlickerPos, 0.0, 1.0);

		explosher.setPosition(Explosher.BLOCKER, debugBlockerPos);
		explosher.setPosition(Explosher.FLICKER, debugFlickerPos);
	}

	private void updateTelemetry()
	{
		Pose pose = follower.pose();

		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose);
		DebugUtil.logAdd("[SNAP] ACTIVE: " + snapActive);
		DebugUtil.logAdd("[SNAP] TARGET: " + activeSnapPoint);
		DebugUtil.logAdd("[SNAP] RANGE: " + SNAP_RANGE);
		DebugUtil.logAdd("[SNAP] FOLLOWING: " + follower.following());

		if (DEBUG_MODE)
		{
			DebugUtil.logAdd("[DEBUG] MODE ON");
			DebugUtil.logAdd("[DEBUG] RPM1 target: " + swagRPM + " current: " + explosher.getCurrentRPM(1));
			DebugUtil.logAdd("[DEBUG] RPM2 target: " + swagRPM2 + " current: " + explosher.getCurrentRPM(2));
			DebugUtil.logAdd("[DEBUG] Blocker target: " + debugBlockerPos + " actual: " + explosher.getPosition(Explosher.BLOCKER));
			DebugUtil.logAdd("[DEBUG] Flicker target: " + debugFlickerPos + " actual: " + explosher.getPosition(Explosher.FLICKER));
			DebugUtil.logAdd("[DEBUG] gamepad1 dpad = RPM | gamepad2 dpad = fingers");
		}
	}
}