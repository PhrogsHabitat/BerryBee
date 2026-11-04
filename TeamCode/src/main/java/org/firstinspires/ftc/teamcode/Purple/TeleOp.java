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

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;


import static com.pedropathing.api.Paths.line;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
		name = "PurpleTeleOp",
		group = "Purple"
)
@Configurable
public class TeleOp extends PurpleOpMode
{

	public static Pose startingPose;

	public static double DEBUG_RPM_STEP = 100.0;
	public static double DEBUG_FINGER_STEP = 0.02;

	// Pollen: 2900 RPM
	// Nectar: 3300 RPM
	public double swagRPM = 2900;
	public double swagRPM2 = 3100;

	public double debugBlockerPos = 0.5;
	public double debugFlickerPos = 0.5;

	private Controls driver1;
	private Controls driver2;

	private Follower follower;
	private Vaccum vaccum;
	private Explosher explosher;

	public static double SNAP_RANGE = 20.0;

	public static Pose[] POLLEN_SNAP_POINTS =
	{
			// Behind
			new Pose(65, 10, Math.toRadians(270)),

			// In Front
			new Pose(48, 124, Math.toRadians(90)),

			// Behind Diagonal
			new Pose(35.5, 19, Math.toRadians(230)),

			// In Front Diagonal
			new Pose(32, 122, Math.toRadians(123)),

	};

	public static Pose[] NECTAR_SNAP_POINTS =
	{
			// Behind
			new Pose(52, 10, Math.toRadians(270)),

			// In Front
			new Pose(70, 135, Math.toRadians(90)),

			// Behind Diagonal
			new Pose(21.5, 24, Math.toRadians(230)),

			// In Front Diagonal
			new Pose(39, 125, Math.toRadians(128)),
	};


	// RPM pair per zone. Index matches POLLEN_SNAP_POINTS / NECTAR_SNAP_POINTS.
	// (pollen[i] and nectar[i] share the same zone, so they share the same RPMs.)
	public static double[] SNAP_RPM_1 = { 2900, 2900, 3000, 3100};
	public static double[] SNAP_RPM_2 = { 2900, 2900, 3000, 3100};

	private boolean snapActive = false;
	private boolean isPollenSnap = false;
	private Pose activeSnapPoint = null;
	private int currentZone = 0;

	private static final FieldManager panelsField =
			PanelsField.INSTANCE.getField();

	private static final Style robotStyle =
			new Style("", "#3F51B5", 2.0);

	private static final double ROBOT_RADIUS = 9.0;

	@Override
	public void create()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);

		PurpleMemory.initialize(hardwareMap, follower);

//		Pose prevPose = new Pose(Integer.parseInt(PurpleMemory.Instance.persist().get("PrevX")), Integer.parseInt(PurpleMemory.Instance.persist().get("PrevY")), Integer.parseInt(PurpleMemory.Instance.persist().get("PrevHeading")));
		follower.setPose(new Pose(9.5, 9.5, Math.toRadians(90)));

		vaccum = new Vaccum(hardwareMap);
		explosher = new Explosher(hardwareMap);

		if (Constants.DEBUG_MODE)
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

		PurpleMemory.Instance.update();

		vaccum.update();
		explosher.update();

		updateDrive();
		updateVaccum();
		updateExplosher();

		follower.update();

		updateTelemetry();
		updatePanelsTelemetry();
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
		boolean rightBumperDown = driver1.isPressed("right_bumper");
		boolean leftBumperDown = driver1.isPressed("left_bumper");

		// Only start a new snap if one isn't already running.
		if (!snapActive)
		{
			if (driver1.justPressed("right_bumper"))
			{
				startSnap(POLLEN_SNAP_POINTS, true);
			}
			else if (driver1.justPressed("left_bumper"))
			{
				startSnap(NECTAR_SNAP_POINTS, false);
			}
		}

		if (snapActive)
		{
			boolean holding = isPollenSnap ? rightBumperDown : leftBumperDown;

			if (holding)
			{
				return;
			}

			follower.stop();
			activeSnapPoint = null;
			snapActive = false;
			isPollenSnap = false;
		}

		double forward = -gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE;
		double strafe = -gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE;
		double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		ManualDrive.driveOrHold(follower, forward, strafe, turn);
	}

	private void startSnap(Pose[] points, boolean pollen)
	{
		Pose currentPose = follower.pose();

		Pose best = null;
		double bestDist = Double.MAX_VALUE;

		for (Pose point : points)
		{
			double dx = point.x() - currentPose.x();
			double dy = point.y() - currentPose.y();
			double distance = Math.sqrt(dx * dx + dy * dy);

			if (distance <= SNAP_RANGE && distance < bestDist)
			{
				bestDist = distance;
				best = point;
			}
		}

		if (best == null)
		{
			return;
		}

		Path snapPath = line(currentPose, best).linear(currentPose, best);

		follower.follow(snapPath);

		activeSnapPoint = best;
		snapActive = true;
		isPollenSnap = pollen;
	}

	private int getClosestZone()
	{
		Pose currentPose = follower.pose();

		int bestIdx = 0;
		double bestDist = Double.MAX_VALUE;

		for (int i = 0; i < POLLEN_SNAP_POINTS.length; i++)
		{
			Pose p = POLLEN_SNAP_POINTS[i];
			Pose n = NECTAR_SNAP_POINTS[i];

			double dp = Math.hypot(p.x() - currentPose.x(), p.y() - currentPose.y());
			double dn = Math.hypot(n.x() - currentPose.x(), n.y() - currentPose.y());
			double d = Math.min(dp, dn);

			if (d < bestDist)
			{
				bestDist = d;
				bestIdx = i;
			}
		}

		return bestIdx;
	}

	private void updateVaccum()
	{
		if (driver2.isPressed("y"))
		{
			vaccum.setPower(1.0);
		}
		else if (driver2.isPressed("x"))
		{
			vaccum.setPower(-0.45);
		}
		else
		{
			vaccum.setPower(0.0);
		}
	}

	private void updateExplosher()
	{
		explosher.setRegressionEnabled(false);

		// Recompute every frame so telemetry (and the auto-RPM below) always reflect truth.
		currentZone = getClosestZone();

		if (Constants.DEBUG_MODE)
		{
			updateExplosherDebug();
			return;
		}

		// Always spin both motors up to the RPM pair of the closest snap zone.
		if (driver2.getRightStickY() > 0.5)
		{
			explosher.setRPM(1, SNAP_RPM_1[currentZone]);
			explosher.setRPM(2, SNAP_RPM_2[currentZone]);
		}
		else if (driver2.getRightStickY() < -0.5)
		{
			explosher.setRPM(1, -2900);
			explosher.setRPM(2, -2900);
		}
		else
		{
			explosher.setRPM(1, 0);
			explosher.setRPM(2, 0);
		}

		boolean snapArrived = snapActive && !follower.following();

		boolean blockerStop =
				driver2.isPressed("right_trigger")
						&& (snapArrived && isPollenSnap);

		boolean flickerStop =
				driver2.isPressed("right_trigger")
						&& (snapArrived && !isPollenSnap);

		explosher.setGateState(
				Explosher.BLOCKER,
				blockerStop ? Explosher.GateState.STOP : Explosher.GateState.PASS
		);
		explosher.setGateState(
				Explosher.FLICKER,
				flickerStop ? Explosher.GateState.STOP : Explosher.GateState.PASS
		);
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

		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose.x() + ", " + pose.y() + ", " + Math.toDegrees(pose.heading()));
		DebugUtil.logAdd("[SNAP] ACTIVE: " + snapActive);
		DebugUtil.logAdd("[SNAP] TYPE: " + (snapActive ? (isPollenSnap ? "POLLEN" : "NECTAR") : "NONE"));
		DebugUtil.logAdd("[SNAP] TARGET: " + activeSnapPoint);
		DebugUtil.logAdd("[SNAP] RANGE: " + SNAP_RANGE);
		DebugUtil.logAdd("[SNAP] FOLLOWING: " + follower.following());

		DebugUtil.logAdd("[ZONE] " + currentZone + " -> RPM1=" + SNAP_RPM_1[currentZone] + " RPM2=" + SNAP_RPM_2[currentZone]);

		if (Constants.DEBUG_MODE)
		{
			DebugUtil.logAdd("[DEBUG] MODE ON");
			DebugUtil.logAdd("[DEBUG] RPM1 target: " + swagRPM + " current: " + explosher.getCurrentRPM(1));
			DebugUtil.logAdd("[DEBUG] RPM2 target: " + swagRPM2 + " current: " + explosher.getCurrentRPM(2));
			DebugUtil.logAdd("[DEBUG] Blocker target: " + debugBlockerPos + " actual: " + explosher.getPosition(Explosher.BLOCKER));
			DebugUtil.logAdd("[DEBUG] Flicker target: " + debugFlickerPos + " actual: " + explosher.getPosition(Explosher.FLICKER));
			DebugUtil.logAdd("[DEBUG] gamepad1 dpad = RPM | gamepad2 dpad = fingers");
		}
	}

	private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private void updatePanelsTelemetry()
	{
		Pose pose = follower.pose();

		if (pose != null)
		{
			double x = pose.x();
			double y = pose.y();
			double heading = pose.heading();

			panelsField.setStyle(robotStyle);

			// Draw the robot's body.
			panelsField.moveCursor(x, y);
			panelsField.circle(ROBOT_RADIUS);

			// Draw a line indicating the robot's heading.
			double headingX = x + Math.cos(heading) * ROBOT_RADIUS;
			double headingY = y + Math.sin(heading) * ROBOT_RADIUS;

			panelsField.moveCursor(x, y);
			panelsField.line(headingX, headingY);

			// Show the robot's current position in text telemetry.
			panelsTelemetry.addData("Robot X", x);
			panelsTelemetry.addData("Robot Y", y);
			panelsTelemetry.addData(
					"Robot Heading (deg)",
					Math.toDegrees(heading)
			);
		}

		panelsTelemetry.addData("Status", "Running");
		panelsTelemetry.addData("Loop Time (ms)", getRuntime());

		// Publish the drawing and telemetry.
		panelsField.update();
		panelsTelemetry.update(telemetry);
	}

}