package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

@Configurable
public class Explosher
{
	public static double CLOSE_SWEET = 0.45;
	public static double FAR_SWEET = 1.0;
	public static double RPM_SMOOTHING_ALPHA = 0.2;
	public static double SHOOTER_KP = 0.0;
	public static double SHOOTER_KV = 0.000155;
	public static double SHOOTER_KS = 0.2;
	public static double RPM_STOP_DEADBAND = 1.0;

	public static double[][] RPM_CALIBRATION_POINTS = {
			{84, 4500},
			{45, 3700},
			{57, 4000},
			{93, 4400},
			{91, 4500},
			{116, 4900}
	};

	public static double AIM_X = 128;
	public static double AIM_Y = 130;

	/** Gate identifiers for the servo gates. */
	public static final int BLOCKER = 1;
	public static final int FLICKER = 2;

	private final MotorConfig motor;
	private final MotorConfig motor2;

	private final ServoConfig blockerConfig;
	private final ServoConfig flickerConfig;

	private GateState blockerState = GateState.STOP;
	private GateState flickerState = GateState.STOP;
	private boolean regressionEnabled = false;
	private boolean hasRegressionTarget = false;
	private double regressionSlope;
	private double regressionIntercept;
	private double smoothedTargetRPM = 0.0;
	private double targetRPM1 = 0.0;
	private double targetRPM2 = 0.0;
	private double aimX = AIM_X;
	private double aimY = AIM_Y;

	public Explosher (HardwareMap hardwareMap)
	{
		motor = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLOSHER,
				MotorConfig.Position.EXPLOSHER,
				112,
				6000
		).zeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
				.build();
		motor.setVelocityDirectionReversed(false);

		motor2 = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLOSHER2,
				MotorConfig.Position.EXPLOSHER,
				112,
				6000
		).zeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
				.build();
		motor2.setVelocityDirectionReversed(false);

		blockerConfig = new ServoConfig.Builder(hardwareMap, Names.NECFINN)
				.setRange(Constants.BLOCKER_MIN, Constants.BLOCKER_MAX)
				.build();
		blockerConfig.setState(ServoConfig.ServoState.ON);

		flickerConfig = new ServoConfig.Builder(hardwareMap, Names.POLLFINN)
				.setRange(Constants.FLICKER_MIN, Constants.FLICKER_MAX)
				.build();
		flickerConfig.setState(ServoConfig.ServoState.ON);

		calculateRegression();
		stop();
		setGateState(BLOCKER, GateState.STOP);
		setGateState(FLICKER, GateState.STOP);
	}

	/**
	 * Updates the shooter and distance-based RPM regression.
	 */
	public void update ()
	{
		updateShooterPower();
		motor.update();
		motor2.update();

		if (regressionEnabled)
		{
			Double targetDistance = getRegressionDistance();
			hasRegressionTarget = targetDistance != null;
			if (hasRegressionTarget)
			{
				double rawTargetRPM = (regressionSlope * targetDistance) + regressionIntercept;
				smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
				smoothedTargetRPM = MathUtil.clamp(smoothedTargetRPM, 0.0, motor.getMaxRPM());
				targetRPM1 = smoothedTargetRPM;
				targetRPM2 = smoothedTargetRPM;
			}
		}
		else
		{
			hasRegressionTarget = false;
		}

		DebugUtil.logAdd("Explosher RPM M1: " +
				String.format("%.2f", getCurrentRPM(1)) +
				" / " +
				String.format("%.2f", getTargetRPM(1)) +
				" | M2: " +
				String.format("%.2f", getCurrentRPM(2)) +
				" / " +
				String.format("%.2f", getTargetRPM(2)));
		DebugUtil.logAdd("Blocker: " + blockerState +
				" | Pos: " + String.format("%.3f", blockerConfig.getPosition()) +
				" || Flicker: " + flickerState +
				" | Pos: " + String.format("%.3f", flickerConfig.getPosition()));
	}

	/**
	 * Stops the shooter motors and clears target RPMs.
	 */
	public void stop ()
	{
		motor.stop();
		motor2.stop();
		targetRPM1 = 0.0;
		targetRPM2 = 0.0;
	}

	public boolean isRegressionEnabled ()
	{
		return regressionEnabled;
	}

	public void setRegressionEnabled (boolean enabled)
	{
		if (regressionEnabled == enabled)
		{
			return;
		}

		regressionEnabled = enabled;
		if (enabled)
		{
			smoothedTargetRPM = getCurrentRPM(1);
		}
	}

	public double getSmoothedTargetRPM ()
	{
		return smoothedTargetRPM;
	}

	public boolean hasRegressionTarget ()
	{
		return hasRegressionTarget;
	}

	/**
	 * Gets odometry-based distance from robot pose to the aim point.
	 *
	 * @return Distance in inches, or null when pose is unavailable.
	 */
	public Double getDistanceToTarget ()
	{
		return getRegressionDistance();
	}

	/**
	 * Sets shooter target RPM for the specified motor (1 or 2).
	 *
	 * @param motorNumber Motor index (1 for motor 1, 2 for motor 2).
	 * @param rpm         Target RPM.
	 */
	public void setRPM (int motorNumber, double rpm)
	{
		if (motorNumber == 1)
		{
			targetRPM1 = MathUtil.clamp(rpm, -motor.getMaxRPM(), motor.getMaxRPM());
		}
		else if (motorNumber == 2)
		{
			targetRPM2 = MathUtil.clamp(rpm, -motor2.getMaxRPM(), motor2.getMaxRPM());
		}
	}

	/**
	 * Sets shooter target RPM for both motors.
	 *
	 * @param rpm Target RPM.
	 */
	public void setRPM (double rpm)
	{
		setRPM(1, rpm);
		setRPM(2, rpm);
	}

	public double getCurrentRPM (int motorNumber)
	{
		if (motorNumber == 2)
		{
			return motor2.getCurrentRPM();
		}
		return motor.getCurrentRPM();
	}

	public double getCurrentRPM ()
	{
		return getCurrentRPM(1);
	}

	public double getTargetRPM (int motorNumber)
	{
		if (motorNumber == 2)
		{
			return targetRPM2;
		}
		return targetRPM1;
	}

	public double getTargetRPM ()
	{
		return getTargetRPM(1);
	}

	public double getMaxRPM ()
	{
		return motor.getMaxRPM();
	}

	/**
	 * Returns the current state of the specified gate.
	 *
	 * @param gate {@link #BLOCKER} or {@link #FLICKER}.
	 */
	public GateState getGateState (int gate)
	{
		return gate == FLICKER ? flickerState : blockerState;
	}

	/**
	 * Sets the state of the specified gate and moves it to the
	 * corresponding STOP or PASS position.
	 *
	 * @param gate  {@link #BLOCKER} or {@link #FLICKER}.
	 * @param state Desired gate state.
	 */
	public void setGateState (int gate, GateState state)
	{
		if (gate == FLICKER)
		{
			flickerState = state;
		}
		else
		{
			blockerState = state;
		}

		ServoConfig config = getGateConfig(gate);
		config.setState(ServoConfig.ServoState.ON);

		switch (state)
		{
			case STOP:
				config.setPosition(getStopPosition(gate));
				break;
			case PASS:
				config.setPosition(getPassPosition(gate));
				break;
		}
	}

	/**
	 * Toggles the specified gate between STOP and PASS.
	 *
	 * @param gate {@link #BLOCKER} or {@link #FLICKER}.
	 */
	public void toggleGateState (int gate)
	{
		setGateState(gate, getGateState(gate) == GateState.STOP ? GateState.PASS : GateState.STOP);
	}

	/**
	 * Returns the raw servo position of the specified gate.
	 *
	 * @param gate {@link #BLOCKER} or {@link #FLICKER}.
	 */
	public double getPosition (int gate)
	{
		return getGateConfig(gate).getPosition();
	}

	/**
	 * Sets the raw servo position of the specified gate.
	 *
	 * @param gate     {@link #BLOCKER} or {@link #FLICKER}.
	 * @param position Servo position.
	 */
	public void setPosition (int gate, double position)
	{
		getGateConfig(gate).setPosition(position);
	}

	private ServoConfig getGateConfig (int gate)
	{
		if (gate == BLOCKER)
		{
			return blockerConfig;
		}
		if (gate == FLICKER)
		{
			return flickerConfig;
		}
		throw new IllegalArgumentException("Gate must be " + BLOCKER + " (blocker) or " + FLICKER + " (flicker).");
	}

	private double getStopPosition (int gate)
	{
		return gate == FLICKER ? Constants.FLICK_POSITION : Constants.BLOCK_POSITION;
	}

	private double getPassPosition (int gate)
	{
		return gate == FLICKER ? Constants.IDLE_POSITION : Constants.PASS_POSITION;
	}

	private void calculateRegression ()
	{
		double[] rpmRegression = calculateLinearRegression(RPM_CALIBRATION_POINTS);
		regressionSlope = rpmRegression[0];
		regressionIntercept = rpmRegression[1];
	}

	private double[] calculateLinearRegression (double[][] calibrationPoints)
	{
		int pointCount = calibrationPoints.length;
		if (pointCount < 2)
		{
			throw new IllegalArgumentException("Need at least 2 calibration points for regression.");
		}

		double sumX = 0.0;
		double sumY = 0.0;
		double sumXY = 0.0;
		double sumX2 = 0.0;

		for (double[] point : calibrationPoints)
		{
			double distance = point[0];
			double rpm = point[1];
			sumX += distance;
			sumY += rpm;
			sumXY += distance * rpm;
			sumX2 += distance * distance;
		}

		double denominator = pointCount * sumX2 - sumX * sumX;
		if (Math.abs(denominator) < 1e-9)
		{
			throw new IllegalArgumentException("Calibration points have invalid X distribution for regression.");
		}

		double slope = (pointCount * sumXY - sumX * sumY) / denominator;
		double intercept = (sumY - slope * sumX) / pointCount;
		return new double[]{slope, intercept};
	}

	private Double getRegressionDistance ()
	{
		if (PurpleMemory.Instance == null)
		{
			return null;
		}

		Pose pose = PurpleMemory.Instance.curPose();
		if (pose == null)
		{
			return null;
		}

		double deltaX = aimX - pose.x();
		double deltaY = aimY - pose.y();
		return Math.hypot(deltaX, deltaY);
	}

	private void updateShooterPower ()
	{
		if (Math.abs(targetRPM1) < RPM_STOP_DEADBAND)
		{
			motor.setPower(0.0);
		}
		else
		{
			double currentRPM1 = motor.getCurrentRPM();
			double rpmError1 = targetRPM1 - currentRPM1;
			double feedforwardPower1 = (SHOOTER_KV * targetRPM1) + Math.copySign(SHOOTER_KS, targetRPM1);
			double correctionPower1 = SHOOTER_KP * rpmError1;
			double commandedPower1 = MathUtil.clamp(feedforwardPower1 + correctionPower1, -1.0, 1.0);
			motor.setPower(commandedPower1);
		}

		if (Math.abs(targetRPM2) < RPM_STOP_DEADBAND)
		{
			motor2.setPower(0.0);
		}
		else
		{
			double currentRPM2 = motor2.getCurrentRPM();
			double rpmError2 = targetRPM2 - currentRPM2;
			double feedforwardPower2 = (SHOOTER_KV * targetRPM2) + Math.copySign(SHOOTER_KS, targetRPM2);
			double correctionPower2 = SHOOTER_KP * rpmError2;
			double commandedPower2 = MathUtil.clamp(feedforwardPower2 + correctionPower2, -1.0, 1.0);
			motor2.setPower(commandedPower2);
		}
	}

	public enum GateState
	{
		STOP,
		PASS
	}
}