package frc.robot.subsystems.shooter;

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
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.RotationConstants;
import frc.robot.Constants.ShooterConstants.WheelConstants;
import frc.robot.util.TalonFXArmSim;
import frc.robot.util.TalonFXSim;

public class ShooterIOSim implements ShooterIO {

  private TalonFXSim wheelMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1), WheelConstants.WHEEL_GEARING, WheelConstants.WHEEL_MOI);

  private TalonFXArmSim hoodSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              1,
              HoodConstants.HOOD_MOI,
              HoodConstants.HOOD_LENGTH.in(Units.Meters),
              HoodConstants.MIN_HOOD_ANGLE.getRadians(),
              HoodConstants.MAX_HOOD_ANGLE.getRadians(),
              false,
              0));

  private TalonFXArmSim rotationSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getKrakenX60Foc(1),
              frc.robot.Constants.ShooterConstants.RotationConstants.GEAR_RATIO,
              RotationConstants.ROTATION_MOI,
              0.5,
              RotationConstants.MIN_ROTATION_ANGLE.getRadians(),
              RotationConstants.MAX_ROTATION_ANGLE.getRadians(),
              false,
              0));

  private VoltageOut wheelOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage wheelClosedLoopControl = new VelocityVoltage(0);

  private VoltageOut hoodOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage hoodClosedLoopControl = new MotionMagicVoltage(0);

  private VoltageOut rotationOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage rotationClosedLoopControl = new MotionMagicVoltage(0);

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterInputs inputs) {
    wheelMotor.update(Constants.kDefaultPeriod);
    inputs.wheelAppliedOutput = wheelMotor.getVoltage();
    inputs.wheelsVelocityRPM = wheelMotor.getVelocity().in(Units.RPM);

    hoodSim.update(Constants.kDefaultPeriod);
    inputs.hoodPosition = new Rotation2d(hoodSim.getPosition());
    inputs.hoodAppliedOutput = hoodSim.getVoltage().in(Units.Volts);
    inputs.hoodVelocity = hoodSim.getVelocity().in(Units.DegreesPerSecond);
    rotationSim.update(Constants.kDefaultPeriod);
    inputs.rotationPosition = new Rotation2d(rotationSim.getPosition());
    inputs.rotationAppliedOutput = rotationSim.getVoltage().in(Units.Volts);
    inputs.rotationVelocity = rotationSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setWheelVoltage(double volts) {
    wheelMotor.setControl(wheelOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setWheelVel(AngularVelocity revPerMin) {
    wheelMotor.setControl(wheelClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void zeroMotors() {
    rotationSim.setState(0, 0);
    hoodSim.setState(0, 0);
  }

  @Override
  public void configWheel(double kV, double kP, double kI, double maxAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kV = kV;

    config.Slot0 = slot0Configs;

    wheelMotor.setConfig(config);
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodSim.setControl(hoodOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setHoodElevation(Rotation2d angle) {
    hoodSim.setControl(hoodClosedLoopControl.withPosition(angle.getRotations()));
  }

  @Override
  public void configHood(double kP, double kI, double kD, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    hoodSim.setConfig(config);
  }

  @Override
  public void setRotationVoltage(double volts) {
    rotationSim.setControl(rotationOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setTurretRotation(Rotation2d angle) {
    rotationSim.setControl(rotationClosedLoopControl.withPosition(angle.getRotations()));
  }

  @Override
  public void configRotation(double kP, double kD, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    rotationSim.setConfig(config);
  }
}
