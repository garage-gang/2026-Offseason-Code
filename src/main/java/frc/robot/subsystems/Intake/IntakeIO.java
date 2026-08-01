package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeInputs {
    public double rollersVelocityRPM;
    public double rollersAppliedOutput;
    public double rollersCurrentAmps;
    public double rollersTempCelsius;
    public double extenderAppliedOutput;
    public double extenderCurrentAmps;
    public double extenderTempCelsius;
    public Rotation2d extenderPosition;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setRollerVoltage(double volts) {}

  public default void setExtenderVoltage(double volts) {}

  public default void setRollerVel(AngularVelocity vel) {}

  public default void setExtenderPos(Rotation2d pos) {}

  public default void configRoller(double kP, double kI, double kV, double maxAcceleration) {}

  public default void configExtender(double kP, double kD, MotionMagicConfigs mmConfigs) {}

  public default void zeroMotors() {}

  public default boolean setExtenderNeutralMode(NeutralModeValue value) {
    return false;
  }
}
