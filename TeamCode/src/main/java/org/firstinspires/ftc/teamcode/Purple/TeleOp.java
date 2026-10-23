package org.firstinspires.ftc.teamcode.Purple;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
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

import java.util.LinkedList;
import java.util.Locale;

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
	private static class PoseRecord
	{
		final double x;
		final double y;
		final double heading;
		final long timeMs;

		PoseRecord(double x, double y, double heading, long timeMs)
		{
			this.x = x;
			this.y = y;
			this.heading = heading;
			this.timeMs = timeMs;
		}
	}

	private static final Pose DEFAULT_TELEOP_START_POSE =
			new Pose(0, 0, 0);

	public static Pose startingPose;

	public static boolean DEBUG_MODE = false;
	public static double DEBUG_RPM_STEP = 100.0;
	public static double DEBUG_FINGER_STEP = 0.02;

	public static double POD_OFFSET_STEP = 0.1;

	public static boolean ENABLE_TRAIL = true;
	public static double TRAIL_LIFETIME_SEC = 3.0;
	public static boolean SHOW_TRAIL_GHOSTS = true;
	public static double TRAIL_LINE_WIDTH = 2.0;
	public static int MAX_TRAIL_POINTS = 300;
	public static double MIN_TRAIL_DISTANCE = 0.1;
	public static double MIN_TRAIL_HEADING = 0.01;

	private double xPodOffset;
	private double yPodOffset;
	private boolean editingXPod = true;
	private final LinkedList<PoseRecord> poseTrail = new LinkedList<>();

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

		PurpleMemory.initialize(hardwareMap);
		loadPodOffsets();

		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);

		follower.setPose(DEFAULT_TELEOP_START_POSE);

		PurpleMemory.initialize(hardwareMap, follower);

		vaccum = new Vaccum(hardwareMap);
		explosher = new Explosher(hardwareMap);

		if (DEBUG_MODE)
		{
			debugBlockerPos = explosher.getPosition(Explosher.BLOCKER);
			debugFlickerPos = explosher.getPosition(Explosher.FLICKER);
		}

		follower.setPose(DEFAULT_TELEOP_START_POSE);
		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update()
	{
		driver1.update();
		driver2.update();

		if (PurpleMemory.Instance != null)
		{
			PurpleMemory.Instance.update();
		}

		vaccum.update();
		explosher.update();

		updateDrive();
		if (!DEBUG_MODE)
		{
			updatePodOffsetTuning();
		}
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
		// Super swag
		boolean snapButtonDown = driver1.isPressed("right_bumper");

		if (driver1.justPressed("start"))
		{
			follower.setPose(new Pose(0, 0, 90));
			poseTrail.clear();
		}

		if (driver1.justPressed("right_bumper"))
		{
			Pose currentPose = follower.pose();

			Pose targetPose = new Pose(0, 0, 0);

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

//		ManualDrive.driveOrHold(follower, forward, strafe, turn);
		follower.manual(forward, strafe, turn);
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

		if (DEBUG_MODE)
		{
			updateExplosherDebug();
			return;
		}

		if (driver2.getRightStickY() > 5.0E-01)
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
			explosher.setGateState(Explosher.FLICKER, Explosher.GateState.PASS);
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

	private void loadPodOffsets()
	{
		if (PurpleMemory.Instance != null && PurpleMemory.Instance.persist() != null)
		{
			String savedX = PurpleMemory.Instance.persist().get("xPodOffset");
			String savedY = PurpleMemory.Instance.persist().get("yPodOffset");

			if (savedX != null)
			{
				try
				{
					xPodOffset = Double.parseDouble(savedX);
					org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.xPodOffset.set(xPodOffset);
				}
				catch (NumberFormatException ignored) {}
			}
			else
			{
				xPodOffset = org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.xPodOffset.get();
			}

			if (savedY != null)
			{
				try
				{
					yPodOffset = Double.parseDouble(savedY);
					org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.yPodOffset.set(yPodOffset);
				}
				catch (NumberFormatException ignored) {}
			}
			else
			{
				yPodOffset = org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.yPodOffset.get();
			}
		}
		else
		{
			xPodOffset = org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.xPodOffset.get();
			yPodOffset = org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.yPodOffset.get();
		}
	}

	private void reinitializeFollower()
	{
		org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.xPodOffset.set(xPodOffset);
		org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig.yPodOffset.set(yPodOffset);

		if (PurpleMemory.Instance != null && PurpleMemory.Instance.persist() != null)
		{
			PurpleMemory.Instance.persist().set("xPodOffset", String.valueOf(xPodOffset));
			PurpleMemory.Instance.persist().set("yPodOffset", String.valueOf(yPodOffset));
		}

		Pose currentPose = (follower != null) ? follower.pose() : DEFAULT_TELEOP_START_POSE;

		if (follower != null)
		{
			follower.stop();
		}

		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
		follower.setPose(currentPose != null ? currentPose : DEFAULT_TELEOP_START_POSE);

		poseTrail.clear();

		PurpleMemory.initialize(hardwareMap, follower);
	}

	private void updatePodOffsetTuning()
	{
		if (driver1.justPressed("dpad_left") || driver1.justPressed("dpad_right"))
		{
			editingXPod = !editingXPod;
		}

		if (driver1.justPressed("dpad_up"))
		{
			if (editingXPod)
			{
				xPodOffset += POD_OFFSET_STEP;
			}
			else
			{
				yPodOffset += POD_OFFSET_STEP;
			}
			reinitializeFollower();
		}
		else if (driver1.justPressed("dpad_down"))
		{
			if (editingXPod)
			{
				xPodOffset -= POD_OFFSET_STEP;
			}
			else
			{
				yPodOffset -= POD_OFFSET_STEP;
			}
			reinitializeFollower();
		}
	}

	private void updateTelemetry()
	{
		Pose pose = follower.pose();

		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose);
		DebugUtil.logAdd("[POD TUNING] Editing: " + (editingXPod ? "X Pod Offset" : "Y Pod Offset") + " (DPad L/R to switch)");
		DebugUtil.logAdd("[POD TUNING] X Pod Offset: " + String.format(Locale.US, "%.3f", xPodOffset));
		DebugUtil.logAdd("[POD TUNING] Y Pod Offset: " + String.format(Locale.US, "%.3f", yPodOffset));
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

	private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private void updatePanelsTelemetry()
	{
		Pose pose = follower.pose();

		if (pose != null)
		{
			double x = pose.x();
			double y = pose.y();
			double heading = pose.heading();

			long now = System.currentTimeMillis();
			long lifetimeMs = Math.max(100L, (long) (TRAIL_LIFETIME_SEC * 1000.0));

			if (ENABLE_TRAIL)
			{
				// Remove expired points based on actual time elapsed so trail disappears
				while (!poseTrail.isEmpty())
				{
					PoseRecord first = poseTrail.peekFirst();
					if (first != null && (now - first.timeMs) > lifetimeMs)
					{
						poseTrail.pollFirst();
					}
					else
					{
						break;
					}
				}

				// Only record point if robot moved or turned enough (or time elapsed)
				boolean shouldAdd = false;
				if (poseTrail.isEmpty())
				{
					shouldAdd = true;
				}
				else
				{
					PoseRecord last = poseTrail.peekLast();
					if (last != null)
					{
						double dist = Math.hypot(x - last.x, y - last.y);
						double headingDiff = Math.abs(MathUtil.clamp(heading - last.heading, -Math.PI, Math.PI));
						if (dist >= MIN_TRAIL_DISTANCE || headingDiff >= MIN_TRAIL_HEADING || (now - last.timeMs) > 100)
						{
							shouldAdd = true;
						}
					}
					else
					{
						shouldAdd = true;
					}
				}

				if (shouldAdd)
				{
					poseTrail.addLast(new PoseRecord(x, y, heading, now));
					while (poseTrail.size() > MAX_TRAIL_POINTS)
					{
						poseTrail.pollFirst();
					}
				}

				// Draw fading line segments
				for (int i = 1; i < poseTrail.size(); i++)
				{
					PoseRecord p1 = poseTrail.get(i - 1);
					PoseRecord p2 = poseTrail.get(i);

					// Skip drawing if pose reset / jumped across the field
					if (Math.hypot(p2.x - p1.x, p2.y - p1.y) > 15.0)
					{
						continue;
					}

					long ageMs = now - p2.timeMs;
					double alphaRatio = 1.0 - ((double) ageMs / lifetimeMs);
					alphaRatio = MathUtil.clamp(alphaRatio, 0.0, 1.0);

					if (alphaRatio <= 0.01) continue;

					int alpha = (int) (alphaRatio * 255);
					// CSS 8-digit hex format: #RRGGBBAA (#3F51B5 + AA)
					String trailColor = String.format(Locale.US, "#3F51B5%02X", alpha);
					double lineWidth = Math.max(0.5, TRAIL_LINE_WIDTH * alphaRatio);

					panelsField.setStyle("", trailColor, lineWidth);
					panelsField.moveCursor(p1.x, p1.y);
					panelsField.line(p2.x, p2.y);
				}

				// Draw fading ghost robot circles and heading indicators
				if (SHOW_TRAIL_GHOSTS && poseTrail.size() > 1)
				{
					int ghostStep = Math.max(1, poseTrail.size() / 12);
					for (int i = 0; i < poseTrail.size(); i += ghostStep)
					{
						PoseRecord p = poseTrail.get(i);
						long ageMs = now - p.timeMs;
						double alphaRatio = 1.0 - ((double) ageMs / lifetimeMs);
						alphaRatio = MathUtil.clamp(alphaRatio, 0.0, 1.0);

						if (alphaRatio <= 0.05) continue;

						int alpha = (int) (alphaRatio * 180);
						String ghostColor = String.format(Locale.US, "#3F51B5%02X", alpha);

						double ghostRadius = ROBOT_RADIUS * 0.4;
						panelsField.setStyle("", ghostColor, 1.0);
						panelsField.moveCursor(p.x, p.y);
						panelsField.circle(ghostRadius);

						double hX = p.x + Math.cos(p.heading) * ghostRadius;
						double hY = p.y + Math.sin(p.heading) * ghostRadius;
						panelsField.moveCursor(p.x, p.y);
						panelsField.line(hX, hY);
					}
				}
			}

			// Draw main active robot body
			panelsField.setStyle(robotStyle);
			panelsField.moveCursor(x, y);
			panelsField.circle(ROBOT_RADIUS);

			// Draw line indicating the robot's heading
			double headingX = x + Math.cos(heading) * ROBOT_RADIUS;
			double headingY = y + Math.sin(heading) * ROBOT_RADIUS;

			panelsField.moveCursor(x, y);
			panelsField.line(headingX, headingY);

			// Show position in text telemetry
			panelsTelemetry.addData("Robot X", x);
			panelsTelemetry.addData("Robot Y", y);
			panelsTelemetry.addData(
					"Robot Heading (deg)",
					Math.toDegrees(heading)
			);
		}

		panelsTelemetry.addData("Editing Pod Offset", editingXPod ? "X Pod" : "Y Pod");
		panelsTelemetry.addData("X Pod Offset (in)", xPodOffset);
		panelsTelemetry.addData("Y Pod Offset (in)", yPodOffset);
		panelsTelemetry.addData("Trail Points", poseTrail.size());
		panelsTelemetry.addData("Status", "Running");
		panelsTelemetry.addData("Loop Time (ms)", getRuntime());

		// Publish the drawing and telemetry.
		panelsField.update();
		panelsTelemetry.update(telemetry);
	}

}