package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Represents a position with x, y coordinates and heading.
 *
 * Supports:
 * - Pedro Pathing 3.0 localization
 * - goBILDA Pinpoint localization
 * - Manually supplied coordinates
 * - Limelight MegaTag vision correction
 */
public class Position
{
	private static final long VISION_CORRECTION_MIN_INTERVAL_MS = 120;
	private static final long VISION_MAX_RESULT_STALENESS_MS = 100;

	private static final double VISION_CORRECTION_BLEND_ALPHA = 0.35;
	private static final double VISION_MAX_REFERENCE_DELTA_INCHES = 30.0;

	private static final int VISION_MIN_TAG_COUNT = 2;
	private static final double VISION_MIN_AVG_AREA = 0.15;
	private static final double VISION_MIN_SINGLE_TAG_AVG_AREA = 0.50;
	private static final double VISION_MAX_AVG_DIST_METERS = 4.0;

	private static final double VISION_MAX_STDDEV_X_METERS = 0.20;
	private static final double VISION_MAX_STDDEV_Y_METERS = 0.20;

	private static final double VISION_FIELD_MIN_INCHES = -8.0;
	private static final double VISION_FIELD_MAX_INCHES = 152.0;

	private static final String DEFAULT_HARDWARE_NAME = "odo";
	private static final double DEFAULT_X_OFFSET_MM = 0.0;
	private static final double DEFAULT_Y_OFFSET_MM = 0.0;

	private static final GoBildaPinpointDriver.GoBildaOdometryPods DEFAULT_POD =
			GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

	private static final GoBildaPinpointDriver.EncoderDirection DEFAULT_FORWARD_DIR =
			GoBildaPinpointDriver.EncoderDirection.FORWARD;

	private static final GoBildaPinpointDriver.EncoderDirection DEFAULT_STRAFE_DIR =
			GoBildaPinpointDriver.EncoderDirection.FORWARD;

	private final GoBildaPinpointDriver odo;
	private final Follower follower;

	private double x;
	private double y;
	private double heading;

	private DistanceUnit distanceUnit = DistanceUnit.INCH;
	private AngleUnit angleUnit = AngleUnit.DEGREES;

	private Pose lastFollowerPose;

	private long lastVisionCorrectionMs = 0;

	private boolean visionCorrectionEnabled = true;

	/**
	 * Constructs a position at (0, 0, 0) with no localization source.
	 */
	public Position()
	{
		odo = null;
		follower = null;

		x = 0.0;
		y = 0.0;
		heading = 0.0;
	}

	/**
	 * Constructs a position with manually supplied coordinates.
	 *
	 * @param x       X coordinate.
	 * @param y       Y coordinate.
	 * @param heading Heading in degrees.
	 */
	public Position(double x, double y, double heading)
	{
		odo = null;
		follower = null;

		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	/**
	 * Constructs a Position backed by Pedro Pathing 3.0.
	 */
	public Position(Follower follower)
	{
		odo = null;
		this.follower = follower;

		x = 0.0;
		y = 0.0;
		heading = 0.0;
	}

	/**
	 * Constructs a Position backed by goBILDA Pinpoint.
	 */
	public Position(HardwareMap hardwareMap)
	{
		this(
				hardwareMap,
				DEFAULT_HARDWARE_NAME,
				DEFAULT_X_OFFSET_MM,
				DEFAULT_Y_OFFSET_MM,
				DistanceUnit.MM,
				DEFAULT_POD,
				DEFAULT_FORWARD_DIR,
				DEFAULT_STRAFE_DIR
		);
	}

	/**
	 * Constructs a Pinpoint-backed Position with custom offsets.
	 */
	public Position(
			HardwareMap hardwareMap,
			String hardwareName,
			double xOffset,
			double yOffset,
			DistanceUnit offsetUnit)
	{
		this(
				hardwareMap,
				hardwareName,
				xOffset,
				yOffset,
				offsetUnit,
				DEFAULT_POD,
				DEFAULT_FORWARD_DIR,
				DEFAULT_STRAFE_DIR
		);
	}

	/**
	 * Constructs a Pinpoint-backed Position with full configuration.
	 */
	public Position(
			HardwareMap hardwareMap,
			String hardwareName,
			double xOffset,
			double yOffset,
			DistanceUnit offsetUnit,
			GoBildaPinpointDriver.GoBildaOdometryPods podType,
			GoBildaPinpointDriver.EncoderDirection forwardDirection,
			GoBildaPinpointDriver.EncoderDirection strafeDirection)
	{
		follower = null;
		odo = hardwareMap.get(
				GoBildaPinpointDriver.class,
				hardwareName
		);

		odo.setOffsets(xOffset, yOffset, offsetUnit);
		odo.setEncoderResolution(podType);
		odo.setEncoderDirections(
				forwardDirection,
				strafeDirection
		);
		odo.resetPosAndIMU();

		x = 0.0;
		y = 0.0;
		heading = 0.0;
	}

	/**
	 * Updates localization from the active source.
	 */
	public void update()
	{
		updateLocalization();
	}

	/**
	 * Updates localization and attempts a vision correction.
	 *
	 * @param result Latest Limelight result.
	 * @return true if a vision correction was applied.
	 */
	public boolean update(LLResult result)
	{
		updateLocalization();

		return applyVisionCorrection(result);
	}

	/**
	 * Updates localization from the active source.
	 */
	private void updateLocalization()
	{
		if (follower != null)
		{
			updateFromFollower();
			return;
		}

		if (odo != null)
		{
			updateFromPinpoint();
		}
	}

	/**
	 * Updates the cached position from Pedro Pathing 3.0.
	 *
	 * Pedro pose values are in inches and radians.
	 */
	private void updateFromFollower()
	{
		Pose pose = follower.pose();

		if (pose == null)
		{
			return;
		}

		x = pose.x();
		y = pose.y();
		heading = Math.toDegrees(pose.heading());

		lastFollowerPose = new Pose(
				pose.x(),
				pose.y(),
				pose.heading()
		);
	}

	/**
	 * Updates the cached position from Pinpoint.
	 */
	private void updateFromPinpoint()
	{
		odo.update();

		Pose2D pose = odo.getPosition();

		x = pose.getX(distanceUnit);
		y = pose.getY(distanceUnit);
		heading = pose.getHeading(angleUnit);
	}

	/**
	 * Enables or disables Limelight vision correction.
	 */
	public void setVisionCorrectionEnabled(boolean enabled)
	{
		visionCorrectionEnabled = enabled;
	}

	/**
	 * Returns whether Limelight vision correction is enabled.
	 */
	public boolean isVisionCorrectionEnabled()
	{
		return visionCorrectionEnabled;
	}

	/**
	 * Attempts to apply a Limelight MegaTag1 localization correction.
	 *
	 * Validation order:
	 *
	 * 1. Validate the Limelight result.
	 * 2. Convert the Limelight pose to Pedro coordinates.
	 * 3. Reject poses outside the accepted field area.
	 * 4. Reject poses too far from Pedro's current estimate.
	 * 5. Blend the translation correction.
	 * 6. Write the corrected pose back into Pedro.
	 *
	 * Heading is preserved from Pedro localization.
	 */
	public boolean applyVisionCorrection(LLResult result)
	{
		if (!visionCorrectionEnabled || follower == null)
		{
			return false;
		}

		VisionPoseCandidate candidate = getVisionPoseCandidate(result);

		if (!passesVisionGate(result, candidate))
		{
			return false;
		}

		Pose visionPose = convertVisionPoseToPedro(candidate.pose3D);

		if (visionPose == null)
		{
			return false;
		}

		/*
		 * Reject impossible vision positions before they can
		 * influence Pedro localization.
		 */
		if (!isLikelyFieldPose(visionPose))
		{
			return false;
		}

		Pose referencePose = getCurrentPedroPose();

		if (referencePose == null)
		{
			return false;
		}

		/*
		 * Reject correction if Pedro's current localization is
		 * already outside the expected field region.
		 */
		if (!isLikelyFieldPose(referencePose))
		{
			return false;
		}

		double deltaX = visionPose.x() - referencePose.x();
		double deltaY = visionPose.y() - referencePose.y();

		double correctionDistance = Math.hypot(deltaX, deltaY);

		/*
		 * Reject large jumps even when both poses are technically
		 * inside the field bounds.
		 */
		if (correctionDistance > VISION_MAX_REFERENCE_DELTA_INCHES)
		{
			return false;
		}

		/*
		 * Blend the vision correction instead of snapping directly
		 * to the Limelight estimate.
		 */
		double correctedX =
				referencePose.x() +
						deltaX * VISION_CORRECTION_BLEND_ALPHA;

		double correctedY =
				referencePose.y() +
						deltaY * VISION_CORRECTION_BLEND_ALPHA;

		/*
		 * Preserve Pedro's heading.
		 */
		double correctedHeading = referencePose.heading();

		Pose correctedPose = new Pose(
				correctedX,
				correctedY,
				correctedHeading
		);

		/*
		 * Final sanity check before modifying Pedro's pose.
		 */
		if (!isLikelyFieldPose(correctedPose))
		{
			return false;
		}

		/*
		 * Write the correction into Pedro's actual localizer.
		 */
		follower.setPose(correctedPose);

		/*
		 * Keep this wrapper synchronized immediately.
		 */
		x = correctedPose.x();
		y = correctedPose.y();
		heading = Math.toDegrees(correctedPose.heading());

		lastFollowerPose = new Pose(
				correctedPose.x(),
				correctedPose.y(),
				correctedPose.heading()
		);

		lastVisionCorrectionMs = System.currentTimeMillis();

		return true;
	}

	/**
	 * Gets the current Pedro pose.
	 */
	private Pose getCurrentPedroPose()
	{
		Pose pose = follower.pose();

		if (pose != null)
		{
			return pose;
		}

		if (lastFollowerPose != null)
		{
			return lastFollowerPose;
		}

		return new Pose(
				x,
				y,
				Math.toRadians(heading)
		);
	}

	/**
	 * Validates a Limelight result.
	 */
	private boolean passesVisionGate(
			LLResult result,
			VisionPoseCandidate candidate)
	{
		if (result == null ||
				candidate == null ||
				!result.isValid())
		{
			return false;
		}

		int tagCount = result.getBotposeTagCount();

		if (tagCount < 1)
		{
			return false;
		}

		List<LLResultTypes.FiducialResult> fiducials =
				result.getFiducialResults();

		if (fiducials == null || fiducials.isEmpty())
		{
			return false;
		}

		if (result.getStaleness() > VISION_MAX_RESULT_STALENESS_MS)
		{
			return false;
		}

		double avgArea = result.getBotposeAvgArea();

		if (avgArea < VISION_MIN_AVG_AREA)
		{
			return false;
		}

		/*
		 * Single-tag measurements require a larger apparent tag area.
		 */
		if (tagCount < VISION_MIN_TAG_COUNT &&
				avgArea < VISION_MIN_SINGLE_TAG_AVG_AREA)
		{
			return false;
		}

		double avgDistance = result.getBotposeAvgDist();

		if (avgDistance <= 0.0 ||
				avgDistance > VISION_MAX_AVG_DIST_METERS)
		{
			return false;
		}

		if (!isStddevWithinLimits(candidate.stddev))
		{
			return false;
		}

		long nowMs = System.currentTimeMillis();

		return nowMs - lastVisionCorrectionMs >=
				VISION_CORRECTION_MIN_INTERVAL_MS;
	}

	/**
	 * Checks whether MegaTag1 standard deviation is acceptable.
	 */
	private boolean isStddevWithinLimits(double[] stddev)
	{
		if (!hasUsableStddev(stddev))
		{
			return false;
		}

		return stddev[0] <= VISION_MAX_STDDEV_X_METERS &&
				stddev[1] <= VISION_MAX_STDDEV_Y_METERS;
	}

	/**
	 * Gets the MegaTag1 pose candidate.
	 */
	private VisionPoseCandidate getVisionPoseCandidate(LLResult result)
	{
		if (result == null)
		{
			return null;
		}

		double[] stddev = result.getStddevMt1();

		if (!hasUsableStddev(stddev))
		{
			return null;
		}

		Pose3D pose3D = result.getBotpose();

		if (pose3D == null)
		{
			return null;
		}

		return new VisionPoseCandidate(
				pose3D,
				stddev
		);
	}

	/**
	 * Checks whether standard deviation data is usable.
	 */
	private boolean hasUsableStddev(double[] stddev)
	{
		if (stddev == null || stddev.length < 2)
		{
			return false;
		}

		if (Double.isNaN(stddev[0]) ||
				Double.isNaN(stddev[1]) ||
				Double.isInfinite(stddev[0]) ||
				Double.isInfinite(stddev[1]))
		{
			return false;
		}

		/*
		 * Limelight may report zero values when the measurement
		 * is unavailable.
		 */
		return stddev[0] > 0.0 || stddev[1] > 0.0;
	}

	/**
	 * Converts the Limelight FTC-coordinate pose to Pedro coordinates.
	 *
	 * FTC -> Pedro:
	 *
	 * Pedro X       = FTC Y + 72
	 * Pedro Y       = 72 - FTC X
	 * Pedro heading = FTC heading - PI / 2
	 */
	private Pose convertVisionPoseToPedro(Pose3D pose3D)
	{
		if (pose3D == null ||
				pose3D.getPosition() == null)
		{
			return null;
		}

		org.firstinspires.ftc.robotcore.external.navigation.Position
				visionPosition = pose3D.getPosition();

		double ftcX = DistanceUnit.INCH.fromUnit(
				visionPosition.unit,
				visionPosition.x
		);

		double ftcY = DistanceUnit.INCH.fromUnit(
				visionPosition.unit,
				visionPosition.y
		);

		double ftcHeading = 0.0;

		if (pose3D.getOrientation() != null)
		{
			ftcHeading = pose3D
					.getOrientation()
					.getYaw(AngleUnit.RADIANS);
		}

		return new Pose(
				ftcY + 72.0,
				72.0 - ftcX,
				ftcHeading - Math.PI / 2.0
		);
	}

	/**
	 * Determines whether a Pedro pose lies within the accepted
	 * field region.
	 */
	private boolean isLikelyFieldPose(Pose pose)
	{
		if (pose == null)
		{
			return false;
		}

		return pose.x() >= VISION_FIELD_MIN_INCHES &&
				pose.x() <= VISION_FIELD_MAX_INCHES &&
				pose.y() >= VISION_FIELD_MIN_INCHES &&
				pose.y() <= VISION_FIELD_MAX_INCHES;
	}

	/**
	 * Calculates XY distance between two Pedro poses.
	 */
	private double distanceXY(Pose a, Pose b)
	{
		double deltaX = a.x() - b.x();
		double deltaY = a.y() - b.y();

		return Math.hypot(deltaX, deltaY);
	}

	/**
	 * Sets the units used by the Position getters.
	 */
	public void setUnits(
			DistanceUnit distanceUnit,
			AngleUnit angleUnit)
	{
		this.distanceUnit = distanceUnit;
		this.angleUnit = angleUnit;
	}

	/**
	 * Gets the X coordinate.
	 */
	public double getX()
	{
		return x;
	}

	/**
	 * Gets the Y coordinate.
	 */
	public double getY()
	{
		return y;
	}

	/**
	 * Gets the heading.
	 */
	public double getHeading()
	{
		return heading;
	}

	/**
	 * Stores a Limelight vision pose and its standard deviation.
	 */
	private static final class VisionPoseCandidate
	{
		private final Pose3D pose3D;
		private final double[] stddev;

		private VisionPoseCandidate(
				Pose3D pose3D,
				double[] stddev)
		{
			this.pose3D = pose3D;
			this.stddev = stddev;
		}
	}
}