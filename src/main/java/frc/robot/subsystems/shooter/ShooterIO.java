package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterInputs {
    public double wheelsVelocityRPM;
    public double wheelAppliedOutput;
    public double wheelCurrentAmps;
    public double wheelTempCelsius;

    public double hoodAppliedOutput;
    public double hoodCurrentAmps;
    public double hoodTempCelsius;
    public double hoodVelocity;
    public Rotation2d hoodPosition;

    public double rotationVelocity;
    public double rotationAppliedOutput;
    public double rotationCurrentAmps;
    public double rotationTempCelsius;
    public Rotation2d rotationPosition;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setWheelVoltage(double volts) {}

  public default void setHoodVoltage(double volts) {}

  public default void setRotationVoltage(double volts) {}

  public default void setWheelVel(AngularVelocity vel) {}

  public default void setHoodElevation(Rotation2d pos) {}

  public default void setTurretRotation(Rotation2d pos) {}

  public default void setRotationVel(AngularVelocity vel) {}

  public default void configWheel(double kV, double kP, double kI, double maxAcceleration) {}

  public default void configHood(double kP, double kI, double kD, MotionMagicConfigs mmConfigs) {}

  public default void configRotation(double kP, double kD, MotionMagicConfigs mmConfigs) {}

  public default void zeroMotors() {}

  public default boolean setHoodNeutralMode(NeutralModeValue value) {
    return false;
  }

  public default boolean setRotationNeutralMode(NeutralModeValue value) {
    return false;
  }
}
