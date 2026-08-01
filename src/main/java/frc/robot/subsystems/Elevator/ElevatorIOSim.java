package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.TalonFXSim;

public class ElevatorIOSim implements ElevatorIO {

  private TalonFXSim elevatorMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          ElevatorConstants.ELEVATOR_GEARING,
          ElevatorConstants.ELEVATOR_MOI);

  private VoltageOut elevatorOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage elevatorClosedLoopControl = new VelocityVoltage(0);

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    elevatorMotor.update(Constants.kDefaultPeriod);
    inputs.elevatorsAppliedOutput = elevatorMotor.getVoltage();
    inputs.elevatorsVelocityRPM = elevatorMotor.getVelocity().in(Units.RPM);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    elevatorMotor.setControl(elevatorOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setElevatorVel(AngularVelocity revPerMin) {
    elevatorMotor.setControl(elevatorClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void configElevator(double kP, double kV, double maxAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;

    config.Slot0 = slot0Configs;
    elevatorMotor.setConfig(config);
  }
}
