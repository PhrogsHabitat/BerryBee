package org.firstinspires.ftc.teamcode.Purple.Components.Lime;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

import java.util.Locale;

@TeleOp(name = "Pollen/Nector Chase", group = "PurpleTests")
@Configurable
public class PollenChaseTest extends PurpleOpMode
{
	public static double TURN_KP = 0.02;
	public static double DRIVE_KP = 0.03;
	public static double DRIVE_PRED_GAIN = 0.02;
	public static double TX_TOLERANCE = 2.0;
	public static double STOP_DIST_IN = 14.0;
	public static double MAX_POWER = 1.0;
	public static double SEARCH_SPIN_POWER = 0.20;
	public static double TX_ALIGN_THRESHOLD = 10.0;
	public static double REVERSE_TURN = -1.0;

	public static double CAMERA_HEIGHT_IN = 13.0;
	public static double TARGET_HEIGHT_IN = 1.8;
	public static double CAMERA_PITCH_DEG = 45.0;
	public static int PIPELINE_INDEX = 1;
	public static boolean AUTO_INTAKE = false;

	private Controls driver1;
	private Controls driver2;

	public MotorConfig fr;
	public MotorConfig fl;
	public MotorConfig br;
	public MotorConfig bl;

	private Follower follower;
	private Vaccum vaccum;

	public boolean fallback = false;
	private boolean autoChaseActive = false;
	private boolean autoChaseToggle = false;

	private int tuneParamIndex = 0;
	private static final String[] TUNE_PARAM_NAMES = {
			"TURN_KP", "DRIVE_KP", "DRIVE_PRED_GAIN", "TX_TOLERANCE", "STOP_DIST_IN",
			"MAX_POWER", "SEARCH_SPIN_POWER", "TX_ALIGN_THRESHOLD", "REVERSE_TURN", "CAMERA_PITCH_DEG"
	};
	private double stepSize = 0.005;
	private final double[] STEP_SIZES = { 0.001, 0.005, 0.01, 0.05, 0.1, 1.0 };
	private int stepSizeIndex = 1;

	private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
	private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

	private static final Style robotStyle = new Style("", "#3F51B5", 2.0);
	private static final double ROBOT_RADIUS = 9.0;

	private double lastSeenTime = -10.0;
	private double lastTx = 0.0;
	private double lastDist = 0.0;
	private double lastUpdateTime = 0.0;
	private double txVelocity = 0.0;
	private double distVelocity = 0.0;
	private double lastTurn = 0.0;
	private double lastForward = 0.0;

	@Override
	public void create()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.EXPLOSHER, 415.2, 1620)
				.zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE).build();
		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.EXPLOSHER, 415.2, 1620)
				.zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE).inverted().build();
		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.EXPLOSHER, 415.2, 1620)
				.zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE).build();
		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.EXPLOSHER, 415.2, 1620)
				.zeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE).inverted().build();

		follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
		PurpleMemory.initialize(hardwareMap, follower);
		follower.setPose(new Pose(14, 90, Math.toRadians(180)));

		vaccum = new Vaccum(hardwareMap);

		LimeUtil.start(hardwareMap, 100);
		LimeUtil.setPipeline(PIPELINE_INDEX);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update()
	{
		driver1.update();
		driver2.update();

		PurpleMemory.Instance.update();
		LimeUtil.update();
		vaccum.update();

		updateRuntimeTuning();
		updateDrive();

		if (!fallback && follower != null)
		{
			follower.update();
		}

		updateVaccum();
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
		LimeUtil.stop();
	}

	private void updateDrive()
	{
		if (driver1.isPressed("y") && driver1.isPressed("start") && driver1.isPressed("right_bumper"))
		{
			fallback = true;
		}

		if (driver1.justPressed("x"))
		{
			autoChaseToggle = !autoChaseToggle;
		}

		boolean holdAutoChase = driver1.isPressed("left_trigger") || driver1.isPressed("a");
		autoChaseActive = autoChaseToggle || holdAutoChase;

		if (autoChaseActive)
		{
			chasePollen();
		}
		else
		{
			double rawForward = -gamepad1.left_stick_y;
			double rawStrafe = -gamepad1.left_stick_x;
			double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

			double forward = rawForward * Constants.DRIVE_POWER_SCALE;
			double strafe = rawStrafe * Constants.DRIVE_POWER_SCALE;

			drive(forward, strafe, turn);
		}
	}

	public boolean chasePollen()
	{
		double currentTime = getRuntime();
		boolean hasTarget = LimeUtil.hasValidTarget();
		LLResult result = hasTarget ? LimeUtil.getResult() : null;
		boolean validResult = result != null && result.isValid();

		if (validResult)
		{
			lastSeenTime = currentTime;
			double tx = result.getTx();
			double ty = result.getTy();
			double dist = distanceToBall(ty);

			double dt = lastUpdateTime > 0.0 ? Math.max(0.01, currentTime - lastUpdateTime) : 0.02;
			txVelocity = (tx - lastTx) / dt;
			distVelocity = lastUpdateTime > 0.0 ? (dist - lastDist) / dt : 0.0;

			lastTx = tx;
			lastDist = dist;
			lastUpdateTime = currentTime;

			double turn = clip(tx * TURN_KP * REVERSE_TURN);
			double distError = dist - STOP_DIST_IN;
			double alignmentFactor = Math.max(0.0, 1.0 - (Math.abs(tx) / 30.0));
			double forward = clip(distError * DRIVE_KP * alignmentFactor);

			lastTurn = turn;
			lastForward = forward;

			drive(forward, 0.0, turn);

			boolean centeredAndAtDistance = Math.abs(tx) < TX_TOLERANCE && Math.abs(distError) <= 2.0;

			if (AUTO_INTAKE && (dist <= STOP_DIST_IN + 4.0) && (Math.abs(tx) < TX_ALIGN_THRESHOLD))
			{
				vaccum.setPower(1.0);
			}

			return centeredAndAtDistance;
		}
		else
		{
			double timeSinceLost = currentTime - lastSeenTime;

			if (timeSinceLost <= 1.5)
			{
				double decay = Math.max(0.0, 1.0 - (timeSinceLost / 1.5));
				double predTurn = clip(lastTurn + (txVelocity * 0.005 * REVERSE_TURN));
				double predForward = clip(Math.max(0.0, lastForward) + (distVelocity * DRIVE_PRED_GAIN));

				drive(predForward * decay, 0.0, predTurn * decay);
			}
			else
			{
				drive(0.0, 0.0, SEARCH_SPIN_POWER);
			}
			return false;
		}
	}

	public double distanceToBall(double ty)
	{
		double deltaH = CAMERA_HEIGHT_IN - TARGET_HEIGHT_IN;
		double angleRad = Math.toRadians(CAMERA_PITCH_DEG - ty);
		if (Math.tan(angleRad) <= 0.0)
		{
			return 0.0;
		}
		return deltaH / Math.tan(angleRad);
	}

	private double clip(double p)
	{
		return Math.max(-MAX_POWER, Math.min(MAX_POWER, p));
	}

	private void drive(double forward, double strafe, double turn)
	{
		if (!fallback && follower != null)
		{
			ManualDrive.driveOrHold(follower, forward, strafe, turn);
		}
		else
		{
			double flPower = forward + strafe + turn;
			double frPower = forward - strafe - turn;
			double blPower = forward - strafe + turn;
			double brPower = forward + strafe - turn;

			fl.setPower(flPower);
			fr.setPower(frPower);
			bl.setPower(blPower);
			br.setPower(brPower);
		}
	}

	private void updateRuntimeTuning()
	{
		if (driver2.justPressed("dpad_up"))
		{
			tuneParamIndex = (tuneParamIndex - 1 + TUNE_PARAM_NAMES.length) % TUNE_PARAM_NAMES.length;
		}
		if (driver2.justPressed("dpad_down"))
		{
			tuneParamIndex = (tuneParamIndex + 1) % TUNE_PARAM_NAMES.length;
		}

		if (driver2.justPressed("right_bumper"))
		{
			stepSizeIndex = (stepSizeIndex + 1) % STEP_SIZES.length;
			stepSize = STEP_SIZES[stepSizeIndex];
		}
		if (driver2.justPressed("left_bumper"))
		{
			stepSizeIndex = (stepSizeIndex - 1 + STEP_SIZES.length) % STEP_SIZES.length;
			stepSize = STEP_SIZES[stepSizeIndex];
		}

		double change = driver2.justPressed("dpad_right") ? stepSize : (driver2.justPressed("dpad_left") ? -stepSize : 0.0);

		if (change != 0.0)
		{
			switch (tuneParamIndex)
			{
				case 0: TURN_KP = Math.max(0.0, TURN_KP + change); break;
				case 1: DRIVE_KP = Math.max(0.0, DRIVE_KP + change); break;
				case 2: DRIVE_PRED_GAIN = Math.max(0.0, DRIVE_PRED_GAIN + change); break;
				case 3: TX_TOLERANCE = Math.max(0.1, TX_TOLERANCE + change); break;
				case 4: STOP_DIST_IN = Math.max(0.0, STOP_DIST_IN + change); break;
				case 5: MAX_POWER = MathUtil.clamp(MAX_POWER + change, 0.05, 1.0); break;
				case 6: SEARCH_SPIN_POWER = MathUtil.clamp(SEARCH_SPIN_POWER + change, -1.0, 1.0); break;
				case 7: TX_ALIGN_THRESHOLD = Math.max(1.0, TX_ALIGN_THRESHOLD + change); break;
				case 8: REVERSE_TURN = (REVERSE_TURN > 0.0) ? -1.0 : 1.0; break;
				case 9: CAMERA_PITCH_DEG = MathUtil.clamp(CAMERA_PITCH_DEG + change, -45.0, 85.0); break;
			}
		}

		if (driver2.justPressed("y"))
		{
			TURN_KP = 0.02;
			DRIVE_KP = 0.03;
			DRIVE_PRED_GAIN = 0.02;
			TX_TOLERANCE = 2.0;
			STOP_DIST_IN = 14.0;
			MAX_POWER = 0.5;
			SEARCH_SPIN_POWER = 0.20;
			TX_ALIGN_THRESHOLD = 10.0;
			REVERSE_TURN = 1.0;
			CAMERA_PITCH_DEG = 20.0;
		}
	}

	private void updateVaccum()
	{
		if (driver2.isPressed("a"))
		{
			vaccum.setPower(1.0);
		}
		else if (driver2.isPressed("b"))
		{
			vaccum.setPower(-0.35);
		}
		else if (!autoChaseActive)
		{
			vaccum.setPower(0.0);
		}
	}

	private void updateTelemetry()
	{
		boolean hasTarget = LimeUtil.hasValidTarget();
		LLResult result = LimeUtil.getResult();
		double tx = (result != null && result.isValid()) ? result.getTx() : 0.0;
		double ty = (result != null && result.isValid()) ? result.getTy() : 0.0;
		double dist = distanceToBall(ty);

		DebugUtil.logAdd("=== LIME POLLEN CHASER ===");
		DebugUtil.logAdd("[MODE] AUTO CHASE: " + (autoChaseActive ? "ACTIVE" : "MANUAL") + " (Hold LT/A or Toggle X)");
		DebugUtil.logAdd("[TARGET] VALID: " + hasTarget + " | TX: " + String.format(Locale.US, "%.2f deg", tx) + " | TY: " + String.format(Locale.US, "%.2f deg", ty));
		DebugUtil.logAdd("[DISTANCE] ESTIMATED: " + String.format(Locale.US, "%.2f in", dist) + " (Stop Dist: " + String.format(Locale.US, "%.1f in", STOP_DIST_IN) + ")");

		DebugUtil.logAdd("--- RUNTIME TUNING (Driver 2 D-Pad & Bumpers) ---");
		DebugUtil.logAdd("SELECTED: > " + TUNE_PARAM_NAMES[tuneParamIndex] + " < (Step: " + stepSize + ")");
		DebugUtil.logAdd(String.format(Locale.US, "TURN_KP: %.4f | DRIVE_KP: %.4f | PRED_GAIN: %.4f", TURN_KP, DRIVE_KP, DRIVE_PRED_GAIN));
		DebugUtil.logAdd(String.format(Locale.US, "TX_TOLERANCE: %.1f deg | STOP_DIST: %.1f in", TX_TOLERANCE, STOP_DIST_IN));
		DebugUtil.logAdd(String.format(Locale.US, "MAX_POWER: %.2f | SEARCH_SPIN: %.2f", MAX_POWER, SEARCH_SPIN_POWER));
		DebugUtil.logAdd(String.format(Locale.US, "ALIGN_THRESH: %.1f deg | PITCH_DEG: %.1f deg", TX_ALIGN_THRESHOLD, CAMERA_PITCH_DEG));
	}

	private void updatePanelsTelemetry()
	{
		Pose pose = follower != null ? follower.pose() : null;

		if (pose != null)
		{
			double x = pose.x();
			double y = pose.y();
			double heading = pose.heading();

			panelsField.setStyle(robotStyle);
			panelsField.moveCursor(x, y);
			panelsField.circle(ROBOT_RADIUS);

			double headingX = x + Math.cos(heading) * ROBOT_RADIUS;
			double headingY = y + Math.sin(heading) * ROBOT_RADIUS;

			panelsField.moveCursor(x, y);
			panelsField.line(headingX, headingY);

			panelsTelemetry.addData("Robot X", x);
			panelsTelemetry.addData("Robot Y", y);
			panelsTelemetry.addData("Robot Heading (deg)", Math.toDegrees(heading));
		}

		panelsTelemetry.addData("Auto Chase", autoChaseActive ? "ACTIVE" : "MANUAL");
		panelsTelemetry.addData("Target Valid", LimeUtil.hasValidTarget());
		panelsTelemetry.addData("Target TX", LimeUtil.getTx());
		panelsTelemetry.addData("Target TY", LimeUtil.getTy());
		panelsTelemetry.addData("Loop Time (ms)", getRuntime());

		panelsField.update();
		panelsTelemetry.update(telemetry);
	}
}
