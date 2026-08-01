package frc.robot.subsystems.Elevator;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorInputs {
    public double elevatorsVelocityRPM;
    public double elevatorsAppliedOutput;
    public double elevatorsCurrentAmps;
    public double elevatorsTempCelsius;
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public default void setElevatorVoltage(double volts) {}

  public default void setElevatorVel(AngularVelocity vel) {}

  public default void configElevator(double kP, double kV, double maxAcceleration) {}
}
