package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO ElevatorIO;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private static final LoggedTunableNumber eleKP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber eleKV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber elevatorTargetAccelerationConfig =
      new LoggedTunableNumber("Elevator/Acceleration");
  private static final LoggedTunableNumber elevatorMaxVelocityConfig =
      new LoggedTunableNumber("Elevator/MaxVelocity");

  static {
    if (Constants.currentMode == Mode.REAL) {
      eleKP.initDefault(0.8);
      eleKV.initDefault(0.15);

      elevatorTargetAccelerationConfig.initDefault(300.0);
      elevatorMaxVelocityConfig.initDefault(3000.0);
    } else {
      eleKP.initDefault(0);
      eleKV.initDefault(0.12);

      elevatorTargetAccelerationConfig.initDefault(500.0);
      elevatorMaxVelocityConfig.initDefault(2000.0);
    }
  }

  public Elevator(ElevatorIO ElevatorIO) {
    this.ElevatorIO = ElevatorIO;
    configElevator();
  }

  public void periodic() {
    Logger.processInputs("Elevator", inputs);
    int hc = hashCode();
    if (eleKP.hasChanged(hc)
        || eleKV.hasChanged(hc)
        || elevatorMaxVelocityConfig.hasChanged(hc)
        || elevatorTargetAccelerationConfig.hasChanged(hc)) configElevator();
    ElevatorIO.updateInputs(inputs);
  }

  public void setElevatorVoltage(double volts) {
    ElevatorIO.setElevatorVoltage(volts);
  }

  public void setElevatorVel(AngularVelocity vel) {
    ElevatorIO.setElevatorVel(vel);
  }

  private void configElevator() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = elevatorTargetAccelerationConfig.get();
    mmConfigs.MotionMagicCruiseVelocity = elevatorMaxVelocityConfig.get();
    ElevatorIO.configElevator(eleKP.get(), eleKV.get(), elevatorMaxVelocityConfig.get());
  }
}
