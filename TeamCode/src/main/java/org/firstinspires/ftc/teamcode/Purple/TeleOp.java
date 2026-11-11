package org.firstinspires.ftc.teamcode.Purple;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import static com.pedropathing.api.Paths.line;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
@Configurable
public class TeleOp extends PurpleOpMode
{

	// Spinup timer and tracking
	private final ElapsedTime explosherSpinupTimer = new ElapsedTime();
	private boolean wasExplosherSpinning = false;
	private boolean wasSnapArrived = false;
	private boolean wasDriver2StickUp = false;
	private boolean wasDriver2StickDown = false;

	public static Pose startingPose;

	public static double DEBUG_RPM_STEP = 100.0;
	public static double DEBUG_FINGER_STEP = 0.02;

	// Pollen: 2900 RPM
	// Nectar: 3300 RPM
	public double swagRPM = 2900;
	public double swagRPM2 = 3100;

	public double debugBlockerPos = 0.5;
	public double debugFlickerPos = 0.5;

	public boolean fallback = false;

	private Controls driver1;
	private Controls driver2;

	public MotorConfig fr;
	public MotorConfig fl;
	public MotorConfig br;
	public MotorConfig bl;

	private Follower follower;
	private Vaccum vaccum;
	private Explosher explosher;

	public static double SNAP_RANGE = 20.0;
	public static boolean FIELD_CENTRIC = false;

	public Pose prevPose;

	// Pose persistence
	private final ElapsedTime saveTimer = new ElapsedTime();
	private static final long SAVE_INTERVAL_MS = 1000;

	// Zero-combo state
	private boolean zeroing = false;

	public static Pose[] POLLEN_SNAP_POINTS =
	{
			// Behind
			new Pose(65, 11, Math.toRadians(270)),

			// In Front
			new Pose(48, 124, Math.toRadians(90)),

			// Behind Diagonal
			new Pose(37, 19, Math.toRadians(230)),

			// In Front Diagonal
			new Pose(32, 122, Math.toRadians(120)),
	};

	public static Pose[] NECTAR_SNAP_POINTS =
	{
			// Behind
			new Pose(52, 11, Math.toRadians(270)),

			// In Front
			new Pose(65, 130, Math.toRadians(90)),

			// Behind Diagonal
			new Pose(21.5, 24, Math.toRadians(230)),

			// In Front Diagonal
			new Pose(39, 125, Math.toRadians(128)),
	};

	private boolean snapActive = false;
	private boolean isPollenSnap = false;
	private Pose activeSnapPoint = null;

	private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
	private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private static final Style robotStyle = new Style("", "#3F51B5", 2.0);

	private static final double ROBOT_RADIUS = 9.0;

	@Override
	public void create()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		// Manual Drive Powers
		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
				.build();

		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
				.inverted()
				.build();

		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
				.build();

		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.EXPLOSHER, 415.2, 1620).zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
				.inverted()
				.build();


		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);

		PurpleMemory.initialize(hardwareMap, follower);

		vaccum = new Vaccum(hardwareMap);
		explosher = new Explosher(hardwareMap);

		if (Constants.DEBUG_MODE)
		{
			debugBlockerPos = explosher.getPosition(Explosher.BLOCKER);
			debugFlickerPos = explosher.getPosition(Explosher.FLICKER);
		}

		DebugUtil.setTelemetry(telemetry);

		try
		{
			prevPose = new Pose(
					Double.parseDouble(PurpleMemory.Instance.persist().get("PrevX", "0")),
					Double.parseDouble(PurpleMemory.Instance.persist().get("PrevY", "0")),
					Math.toRadians(Double.parseDouble(PurpleMemory.Instance.persist().get("PrevHeading", "0")))
			);

//			follower.setPose(new Pose(prevPose.x(), prevPose.y(), prevPose.heading()));
			follower.setPose(new Pose(14, 90, Math.toRadians(180)));

		}
		catch(Exception e)
		{
			DebugUtil.logAdd(e.getMessage());
			follower.setPose(new Pose(14, 90, Math.toRadians(180)));
		}

		saveTimer.reset();
	}

	@Override
	public void update()
	{
		driver1.update();
		driver2.update();

		PurpleMemory.Instance.update();

		vaccum.update();
		explosher.update();

		// Handle switching to fallback controls
		if (!fallback)
		{
			updateDrive();
			follower.update();
		}
		else
		{
			updateFallbackDrive();
		}

		updateVaccum();
		updateExplosher();

		updateTelemetry();
		updatePanelsTelemetry();
		DebugUtil.update();
	}

	@Override
	public void destroy()
	{
		// Persist the current pose before shutting anything down.
		savePose();

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

		boolean fallbackCombo = driver1.isPressed("y") && driver1.isPressed("start") && driver1.isPressed("right_bumper");
		boolean zeroCombo = driver1.isPressed("y") && driver1.isPressed("start") && !driver1.isPressed("right_bumper");

		if (fallbackCombo)
		{
			fallback = true;
			return;
		}

		if (zeroCombo)
		{
			if (!zeroing)
			{
				// Cancel any active snap.
				snapActive = false;
				activeSnapPoint = null;
				isPollenSnap = false;

				// Clear the latched hold target, reset the pose, and re-latch
				// the hold at the new origin so driveOrHold has nothing to chase.
				follower.stop();
				follower.setPose(new Pose(10, 132, 270));
				follower.hold(new Pose(10, 132, 270));
				zeroing = true;
			}

			// While the combo is held, keep the hold target pinned to origin
			// and skip driveOrHold entirely.
			follower.hold(new Pose(0, 0, 0));
			return;
		}

		if (zeroing)
		{
			// Input so it drops any internally-latched hold target
			zeroing = false;
			ManualDrive.driveOrHold(follower, 0.05, 0.05, 0.05);
			return;
		}

		// Only start a new snap if one isn't already running.
		if (!snapActive)
		{
			if (driver1.justPressed("left_bumper"))
			{
				startSnap(POLLEN_SNAP_POINTS, true);
			}
			else if (driver1.justPressed("right_bumper"))
			{
				startSnap(NECTAR_SNAP_POINTS, false);
			}
		}

		if (snapActive)
		{
			boolean snapArrived = !follower.isBusy();
			if (snapArrived && !wasSnapArrived)
			{
				driver1.vibrate(500);
			}
			wasSnapArrived = snapArrived;

			boolean holding = isPollenSnap ? leftBumperDown : rightBumperDown;

			if (holding)
			{
				return;
			}

			follower.stop();
			activeSnapPoint = null;
			snapActive = false;
			isPollenSnap = false;
			wasSnapArrived = false;
		}
		else
		{
			wasSnapArrived = false;
		}

		double rawForward = -gamepad1.left_stick_y;
		double rawStrafe = -gamepad1.left_stick_x;
		double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		double forward;
		double strafe;

		if (FIELD_CENTRIC)
		{
			Pose currentPose = follower.pose();
			if (currentPose != null)
			{
				double heading = currentPose.heading();
				forward = rawForward * Math.cos(heading) + rawStrafe * Math.sin(heading);
				strafe = -rawForward * Math.sin(heading) + rawStrafe * Math.cos(heading);
			}
			else
			{
				forward = rawForward;
				strafe = rawStrafe;
			}
		}
		else
		{
			forward = rawForward;
			strafe = rawStrafe;
		}

		forward *= Constants.DRIVE_POWER_SCALE;
		strafe *= Constants.DRIVE_POWER_SCALE;

		ManualDrive.driveOrHold(follower, forward, strafe, turn);
	}

	private void updateFallbackDrive()
	{

		double rawForward = -gamepad1.left_stick_y;
		double rawStrafe = gamepad1.left_stick_x;
		double turn = gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		double forward = rawForward;
		double strafe = rawStrafe;

		forward *= Constants.DRIVE_POWER_SCALE;
		strafe *= Constants.DRIVE_POWER_SCALE;

		double flPower  = forward + strafe + turn;
		double frPower = forward - strafe - turn;
		double blPower   = forward - strafe + turn;
		double brPower  = forward + strafe - turn;

		fl.setPower(flPower);
		fr.setPower(frPower);
		bl.setPower(blPower);
		br.setPower(brPower);

	}

	private void startSnap(Pose[] points, boolean pollen)
	{
		Pose currentPose = follower.pose();
		if (currentPose == null)
		{
			return;
		}

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

		follower.hold(best);

		activeSnapPoint = best;
		snapActive = true;
		isPollenSnap = pollen;
	}

	private void savePose()
	{
		if (follower == null || PurpleMemory.Instance == null
				|| PurpleMemory.Instance.persist() == null)
		{
			return;
		}

		Pose p = follower.pose();
		if (p == null)
		{
			return;
		}

		// Batched so X/Y/Heading are written in a single atomic transaction.
		Map<String, String> values = new HashMap<>();
		values.put("PrevX", String.valueOf(p.x()));
		values.put("PrevY", String.valueOf(p.y()));
		values.put("PrevHeading", String.valueOf(Math.toDegrees(p.heading())));
		PurpleMemory.Instance.persist().setAll(values);
	}

	private void updateVaccum()
	{
		if (driver2.isPressed("y"))
		{
			vaccum.setPower(1.0);
		}
		else if (driver2.isPressed("x"))
		{
			vaccum.setPower(-0.35);
		}
		else
		{
			vaccum.setPower(0.0);
		}
	}

	private void updateExplosher()
	{
		explosher.setRegressionEnabled(false);

		if (Constants.DEBUG_MODE)
		{
			updateExplosherDebug();
			return;
		}

		// Spin motors to target RPM with vibration on initial push.
		double rightStickY = driver2.getRightStickY();
		boolean stickUp = rightStickY > 0.5;
		boolean stickDown = rightStickY < -0.5;

		if (stickUp && !wasDriver2StickUp)
		{
			driver2.vibrate(250);
			explosher.setRPM(Explosher.TARGET_RPM);
		}
		else if (stickDown && !wasDriver2StickDown)
		{
			driver2.vibrate(250);
			explosher.setRPM(-Explosher.TARGET_RPM);
		}
		else if (!stickUp && !stickDown)
		{
			explosher.setRPM(0);
		}
		else
		{
			if (stickUp)
			{
				explosher.setRPM(Explosher.TARGET_RPM);
			}
			else
			{
				explosher.setRPM(-Explosher.TARGET_RPM);
			}
		}

		wasDriver2StickUp = stickUp;
		wasDriver2StickDown = stickDown;

		boolean isSpinning = Math.abs(explosher.getTargetRPM()) > 10.0;
		if (isSpinning)
		{
			if (!wasExplosherSpinning)
			{
				explosherSpinupTimer.reset();
				wasExplosherSpinning = true;
			}
		}
		else
		{
			wasExplosherSpinning = false;
			explosherSpinupTimer.reset();
		}

		boolean explosherReady = wasExplosherSpinning && (explosherSpinupTimer.seconds() >= 2.0);

		boolean rightTriggerPressed = driver2.isPressed("right_trigger");

		if (driver2.justPressed("right_trigger"))
		{
			driver2.vibrate(10);
		}

		boolean snapArrived = snapActive && !follower.isBusy();

		boolean blockerStop;
		boolean flickerStop;

		if (!fallback)
		{
			blockerStop =
					rightTriggerPressed
							&& explosherReady
							&& (snapArrived && isPollenSnap);

			flickerStop =
					rightTriggerPressed
							&& explosherReady
							&& (snapArrived && !isPollenSnap);
		}
		else
		{
			blockerStop = driver2.isPressed("left_bumper");
			flickerStop = driver2.isPressed("right_bumper");
		}

		boolean forceOut = driver2.isPressed("left_trigger");

		explosher.setGateState(
				Explosher.BLOCKER,
				forceOut ? Explosher.GateState.STOP : blockerStop ? Explosher.GateState.STOP : Explosher.GateState.PASS
		);
		explosher.setGateState(
				Explosher.FLICKER,
				forceOut ? Explosher.GateState.STOP : flickerStop ? Explosher.GateState.STOP : Explosher.GateState.PASS
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

		try
		{
			DebugUtil.logAdd("[PREV SWAG] " + prevPose.x() + ", " + prevPose.y() + ", " + prevPose.heading());
		}
		catch (Exception e)
		{
			DebugUtil.logAdd(e.getMessage());
		}

		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("[VOLTAGE] " + explosher.getVoltage());

		if (pose != null)
		{
			DebugUtil.logAdd("[FOLLOWER] POSE: " + pose.x() + ", " + pose.y() + ", " + Math.toDegrees(pose.heading()));
		}
		else
		{
			DebugUtil.logAdd("[FOLLOWER] POSE: null");
		}

		DebugUtil.logAdd("[SNAP] ACTIVE: " + snapActive);
		DebugUtil.logAdd("[SNAP] TYPE: " + (snapActive ? (isPollenSnap ? "POLLEN" : "NECTAR") : "NONE"));
		DebugUtil.logAdd("[SNAP] TARGET: " + activeSnapPoint);
		DebugUtil.logAdd("[SNAP] RANGE: " + SNAP_RANGE);
		DebugUtil.logAdd("[SNAP] FOLLOWING: " + follower.following());
		DebugUtil.logAdd("[ZERO] ACTIVE: " + zeroing);

		DebugUtil.logAdd("[BATTERY RPM] TARGET=" + String.format(Locale.US, "%.1f", explosher.getBatteryAdjustedRPM()) + " (BASE=" + Explosher.TARGET_RPM + ", VOLTAGE=" + String.format(Locale.US, "%.2f", explosher.getVoltage()) + "V)");

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