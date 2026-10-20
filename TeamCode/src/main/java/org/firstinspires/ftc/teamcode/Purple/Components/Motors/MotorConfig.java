
package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class MotorConfig
{
    private final String name;
    private final Position position;
    private final MotorEx motor;
    private final double maxRPM;
    private final double cpr;

    private double velocitySign = 1.0;
    private ControlMode controlMode = ControlMode.RAW_POWER;
    private double targetRPM = 0.0;
    private double targetPower = 0.0;

    private MotorConfig(Builder builder)
    {
        if (builder.hardwareMap == null)
        {
            throw new IllegalArgumentException("HardwareMap cannot be null.");
        }

        if (builder.name == null || builder.name.trim().isEmpty())
        {
            throw new IllegalArgumentException("Motor name cannot be empty.");
        }

        if (!Double.isFinite(builder.cpr) || builder.cpr < 0.0
                || !Double.isFinite(builder.maxRPM) || builder.maxRPM < 0.0)
        {
            throw new IllegalArgumentException("CPR and max RPM must be finite and nonnegative.");
        }

        if ((builder.cpr > 0.0) != (builder.maxRPM > 0.0))
        {
            throw new IllegalArgumentException(
                    "CPR and max RPM must both be positive for velocity control.");
        }

        name = builder.name;
        position = builder.position;
        cpr = builder.cpr;
        maxRPM = builder.maxRPM;

        motor = cpr > 0.0
                ? new MotorEx(builder.hardwareMap, name, cpr, maxRPM)
                : new MotorEx(builder.hardwareMap, name);

        motor.setInverted(builder.inverted);
        motor.setZeroPowerBehavior(builder.zeroPowerBehavior);

        if (builder.positionCoefficient > 0.0)
        {
            motor.setPositionCoefficient(builder.positionCoefficient);
        }

        if (builder.positionTolerance >= 0.0)
        {
            motor.setPositionTolerance(builder.positionTolerance);
        }

        if (builder.distancePerPulse > 0.0 && cpr > 0.0)
        {
            motor.setDistancePerPulse(builder.distancePerPulse);
        }

        if (builder.velocityGainsConfigured)
        {
            motor.setVeloCoefficients(
                    builder.velocityKp,
                    builder.velocityKi,
                    builder.velocityKd
            );
        }

        if (builder.feedforwardConfigured)
        {
            motor.setFeedforwardCoefficients(
                    builder.feedforwardKs,
                    builder.feedforwardKv
            );
        }

        setControlMode(
                builder.velocityEnabled && cpr > 0.0 && maxRPM > 0.0
                        ? ControlMode.VELOCITY_CONTROL
                        : ControlMode.RAW_POWER
        );
    }

    private double rpmToTps(double rpm)
    {
        return cpr > 0.0 ? rpm * cpr / 60.0 : 0.0;
    }

    private double tpsToRpm(double tps)
    {
        return cpr > 0.0 ? tps * 60.0 / cpr : 0.0;
    }

    private double applyVelocityDirection(double value)
    {
        return value * velocitySign;
    }

    private static double clamp(double value, double min, double max)
    {
        return Math.max(min, Math.min(max, value));
    }

    private static void requireFinite(double value, String label)
    {
        if (!Double.isFinite(value))
        {
            throw new IllegalArgumentException(label + " must be finite.");
        }
    }

    public String getName()
    {
        return name;
    }

    public Position getPosition()
    {
        return position;
    }

    public double getMaxRPM()
    {
        return maxRPM;
    }

    public double getCPR()
    {
        return cpr;
    }

    public ControlMode getControlMode()
    {
        return controlMode;
    }

    public void setControlMode(ControlMode mode)
    {
        if (mode == null)
        {
            throw new IllegalArgumentException("Control mode cannot be null.");
        }

        if (mode == ControlMode.VELOCITY_CONTROL && (cpr <= 0.0 || maxRPM <= 0.0))
        {
            throw new IllegalStateException(
                    "Velocity control requires positive CPR and max RPM.");
        }

        controlMode = mode;

        switch (mode)
        {
            case VELOCITY_CONTROL:
                motor.setRunMode(Motor.RunMode.VelocityControl);
                break;

            case POSITION_CONTROL:
                motor.setRunMode(Motor.RunMode.PositionControl);
                break;

            case RAW_POWER:
            default:
                motor.setRunMode(Motor.RunMode.RawPower);
                break;
        }
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }

    public void setTargetRPM(double rpm)
    {
        requireFinite(rpm, "Target RPM");

        if (controlMode != ControlMode.VELOCITY_CONTROL)
        {
            if (cpr <= 0.0 || maxRPM <= 0.0)
            {
                throw new IllegalStateException(
                        "RPM commands require configured CPR and max RPM.");
            }

            setControlMode(ControlMode.VELOCITY_CONTROL);
        }

        targetRPM = clamp(rpm, -maxRPM, maxRPM);

        double targetTps = applyVelocityDirection(rpmToTps(targetRPM));

        // FTCLib VelocityControl uses set() to run its own controller.
        // Do not use setVelocity() here; that bypasses this control path.
        motor.set(targetTps);
    }

    public double getCurrentRPM()
    {
        return tpsToRpm(getVelocity());
    }

    public double getVelocity()
    {
        return applyVelocityDirection(motor.getVelocity());
    }

    public void setVelocityDirectionReversed(boolean reversed)
    {
        velocitySign = reversed ? -1.0 : 1.0;
    }

    public boolean isVelocityDirectionReversed()
    {
        return velocitySign < 0.0;
    }

    public double getPower()
    {
        return motor.get();
    }

    public void setPower(double power)
    {
        requireFinite(power, "Power");

        targetPower = clamp(power, -1.0, 1.0);

        if (controlMode != ControlMode.RAW_POWER)
        {
            setControlMode(ControlMode.RAW_POWER);
        }

        targetRPM = maxRPM > 0.0 ? targetPower * maxRPM : 0.0;
        motor.set(targetPower);
    }

    public double getPositionCoefficient()
    {
        return motor.getPositionCoefficient();
    }

    public void setPositionCoefficient(double coefficient)
    {
        requireFinite(coefficient, "Position coefficient");

        if (coefficient < 0.0)
        {
            throw new IllegalArgumentException(
                    "Position coefficient cannot be negative.");
        }

        motor.setPositionCoefficient(coefficient);
    }

    public void setTargetPosition(int target)
    {
        motor.setTargetPosition(target);
    }

    public void setPositionTolerance(double tolerance)
    {
        requireFinite(tolerance, "Position tolerance");

        if (tolerance < 0.0)
        {
            throw new IllegalArgumentException(
                    "Position tolerance cannot be negative.");
        }

        motor.setPositionTolerance(tolerance);
    }

    public boolean atTargetPosition()
    {
        return motor.atTargetPosition();
    }

    public int getCurrentPosition()
    {
        return motor.getCurrentPosition();
    }

    public void resetEncoder()
    {
        motor.resetEncoder();
    }

    public void setDistancePerPulse(double distancePerPulse)
    {
        requireFinite(distancePerPulse, "Distance per pulse");

        if (distancePerPulse <= 0.0)
        {
            throw new IllegalArgumentException(
                    "Distance per pulse must be positive.");
        }

        if (cpr <= 0.0)
        {
            throw new IllegalStateException(
                    "Distance per pulse requires configured CPR.");
        }

        motor.setDistancePerPulse(distancePerPulse);
    }

    public void setTargetDistance(double distance)
    {
        requireFinite(distance, "Target distance");
        motor.setTargetDistance(distance);
    }

    public double getDistance()
    {
        return motor.getDistance();
    }

    public void runToPosition(int targetPosition, double power)
    {
        requireFinite(power, "Position power");

        if (controlMode != ControlMode.POSITION_CONTROL)
        {
            setControlMode(ControlMode.POSITION_CONTROL);
        }

        targetPower = clamp(Math.abs(power), 0.0, 1.0);
        setTargetPosition(targetPosition);
        motor.set(targetPower);
    }

    public void runToDistance(double targetDistance, double power)
    {
        requireFinite(power, "Position power");

        if (controlMode != ControlMode.POSITION_CONTROL)
        {
            setControlMode(ControlMode.POSITION_CONTROL);
        }

        targetPower = clamp(Math.abs(power), 0.0, 1.0);
        setTargetDistance(targetDistance);
        motor.set(targetPower);
    }

    public void stop()
    {
        motor.stopMotor();
        targetRPM = 0.0;
        targetPower = 0.0;
    }

    public void update()
    {
        // FTCLib manages velocity control internally through set().
        // Do not repeatedly overwrite velocity commands here.
        if (controlMode == ControlMode.POSITION_CONTROL)
        {
            if (atTargetPosition())
            {
                motor.set(0.0);
            }
            else
            {
                motor.set(targetPower);
            }
        }
    }

    public enum Position
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        EXPLOSHER,
        INTAKE,
        MIDTAKE
    }

    public enum ControlMode
    {
        RAW_POWER,
        VELOCITY_CONTROL,
        POSITION_CONTROL
    }

    public static class Builder
    {
        private final HardwareMap hardwareMap;
        private final String name;
        private final Position position;
        private final double maxRPM;
        private final double cpr;

        private boolean inverted = false;
        private Motor.ZeroPowerBehavior zeroPowerBehavior =
                Motor.ZeroPowerBehavior.BRAKE;

        private boolean velocityEnabled = true;

        private double positionCoefficient = 0.05;
        private double positionTolerance = 13.6;
        private double distancePerPulse = 0.0;

        private boolean velocityGainsConfigured = false;
        private double velocityKp = 0.0;
        private double velocityKi = 0.0;
        private double velocityKd = 0.0;

        private boolean feedforwardConfigured = false;
        private double feedforwardKs = 0.0;
        private double feedforwardKv = 0.0;

        public Builder(
                HardwareMap hw,
                String name,
                Position position,
                double cpr,
                double maxRPM)
        {
            this.hardwareMap = hw;
            this.name = name;
            this.position = position;
            this.cpr = cpr;
            this.maxRPM = maxRPM;
        }

        public Builder(HardwareMap hw, String name, Position position)
        {
            this(hw, name, position, 0.0, 0.0);
        }

        public Builder inverted()
        {
            inverted = true;
            return this;
        }

        public Builder zeroPowerBehavior(
                Motor.ZeroPowerBehavior zeroPowerBehavior)
        {
            if (zeroPowerBehavior == null)
            {
                throw new IllegalArgumentException(
                        "Zero power behavior cannot be null.");
            }

            this.zeroPowerBehavior = zeroPowerBehavior;
            return this;
        }

        public Builder disableVelocityControl()
        {
            velocityEnabled = false;
            return this;
        }

        public Builder setPositionCoefficient(double coefficient)
        {
            requireFinite(coefficient, "Position coefficient");

            if (coefficient < 0.0)
            {
                throw new IllegalArgumentException(
                        "Position coefficient cannot be negative.");
            }

            positionCoefficient = coefficient;
            return this;
        }

        public Builder setPositionTolerance(double tolerance)
        {
            requireFinite(tolerance, "Position tolerance");

            if (tolerance < 0.0)
            {
                throw new IllegalArgumentException(
                        "Position tolerance cannot be negative.");
            }

            positionTolerance = tolerance;
            return this;
        }

        public Builder setDistancePerPulse(double distancePerPulse)
        {
            requireFinite(distancePerPulse, "Distance per pulse");

            if (distancePerPulse <= 0.0)
            {
                throw new IllegalArgumentException(
                        "Distance per pulse must be positive.");
            }

            this.distancePerPulse = distancePerPulse;
            return this;
        }

        public Builder setVelocityPID(double kp, double ki, double kd)
        {
            requireFinite(kp, "Velocity kP");
            requireFinite(ki, "Velocity kI");
            requireFinite(kd, "Velocity kD");

            if (kp < 0.0 || ki < 0.0 || kd < 0.0)
            {
                throw new IllegalArgumentException(
                        "Velocity PID gains cannot be negative.");
            }

            velocityKp = kp;
            velocityKi = ki;
            velocityKd = kd;
            velocityGainsConfigured = true;
            return this;
        }

        public Builder setVelocityFeedforward(double ks, double kv)
        {
            requireFinite(ks, "Feedforward kS");
            requireFinite(kv, "Feedforward kV");

            feedforwardKs = ks;
            feedforwardKv = kv;
            feedforwardConfigured = true;
            return this;
        }

        public MotorConfig build()
        {
            return new MotorConfig(this);
        }
    }
}
