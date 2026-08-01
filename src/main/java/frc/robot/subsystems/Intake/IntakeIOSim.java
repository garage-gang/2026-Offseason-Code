package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.ExtenderConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.util.TalonFXArmSim;
import frc.robot.util.TalonFXSim;

public class IntakeIOSim implements IntakeIO {

  private TalonFXSim rollerMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1), RollerConstants.ROLLER_GEARING, RollerConstants.ROLLER_MOI);

  private TalonFXArmSim extenderSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              frc.robot.Constants.IntakeConstants.ExtenderConstants.GEAR_RATIO,
              ExtenderConstants.EXTENDER_MOI,
              ExtenderConstants.EXTENDER_LENGTH.in(Units.Meters),
              ExtenderConstants.MIN_EXTENDER_ANGLE.getRadians(),
              ExtenderConstants.MAX_EXTENDER_ANGLE.getRadians(),
              false,
              0));

  private VoltageOut rollerOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage rollerClosedLoopControl = new VelocityVoltage(0);

  private VoltageOut extenderOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage extenderClosedLoopControl = new MotionMagicVoltage(0);

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeInputs inputs) {
    rollerMotor.update(Constants.kDefaultPeriod);
    inputs.rollersAppliedOutput = rollerMotor.getVoltage();
    inputs.rollersVelocityRPM = rollerMotor.getVelocity().in(Units.RPM);

    extenderSim.update(Constants.kDefaultPeriod);

    inputs.extenderPosition = new Rotation2d(extenderSim.getPosition());
    inputs.extenderAppliedOutput = extenderSim.getVoltage().in(Units.Volts);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVel(AngularVelocity revPerMin) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void configRoller(double kP, double kI, double kV, double maxAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kV = kV;

    config.Slot0 = slot0Configs;

    mmConfig.MotionMagicAcceleration = 1000;

    rollerMotor.setConfig(config);
  }

  @Override
  public void setExtenderVoltage(double volts) {
    extenderSim.setControl(extenderOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setExtenderPos(Rotation2d angle) {
    extenderSim.setControl(extenderClosedLoopControl.withPosition(angle.getRotations()));
  }

  @Override
  public void configExtender(double kP, double kD, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    extenderSim.setConfig(config);
  }

  @Override
  public void zeroMotors() {
    extenderSim.setState(0, 0);
  }
}
