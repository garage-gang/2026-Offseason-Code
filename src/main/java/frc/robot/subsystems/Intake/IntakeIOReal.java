package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.ExtenderConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;

public class IntakeIOReal implements IntakeIO {

  private final TalonFX rollerMotor = new TalonFX(Constants.CanIDs.INTAKE_ROLLER_CAN_ID);
  private final TalonFX extenderMotor = new TalonFX(Constants.CanIDs.INTAKE_EXTENDER_CAN_ID);

  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerDeviceTemp;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Temperature> extenderDeviceTemp;
  private final StatusSignal<Voltage> extenderAppliedVoltage;
  private final StatusSignal<Angle> extenderAngle;
  private final StatusSignal<Current> extenderCurrent;

  private final VoltageOut rollerOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut extenderOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage rollerClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage extenderClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  public IntakeIOReal() {
    // Motor config
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimitConfig = new CurrentLimitsConfigs();

    rollerCurrentLimitConfig.SupplyCurrentLimit = RollerConstants.SUPPLY_CURRENT_LIMIT;
    rollerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    rollerCurrentLimitConfig.StatorCurrentLimit = RollerConstants.STATOR_CURRENT_LIMIT;
    rollerCurrentLimitConfig.StatorCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimitConfig;

    rollerConfig.Feedback.SensorToMechanismRatio = RollerConstants.ROLLER_GEARING;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfig.Voltage.SupplyVoltageTimeConstant = RollerConstants.SUPPLY_VOLTAGE_TIME;

    rollerMotor.getConfigurator().apply(rollerConfig);

    // Status signals

    rollerCurrent = rollerMotor.getStatorCurrent();
    rollerDeviceTemp = rollerMotor.getDeviceTemp();
    rollerAppliedVoltage = rollerMotor.getMotorVoltage();
    rollerVelocity = rollerMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, rollerAppliedVoltage, rollerCurrent, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, rollerDeviceTemp);

    rollerMotor.optimizeBusUtilization();

    TalonFXConfiguration extenderConfig = new TalonFXConfiguration();

    extenderConfig.CurrentLimits.SupplyCurrentLimit = ExtenderConstants.SUPPLY_CURRENT_LIMIT;
    extenderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extenderConfig.CurrentLimits.StatorCurrentLimit = ExtenderConstants.STATOR_CURRENT_LIMIT;
    extenderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extenderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extenderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ExtenderConstants.MAX_EXTENDER_ANGLE.getRotations();
    extenderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ExtenderConstants.MIN_EXTENDER_ANGLE.minus(Rotation2d.fromDegrees(2.0)).getRotations();

    extenderConfig.Feedback.SensorToMechanismRatio = ExtenderConstants.GEAR_RATIO;
    extenderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    extenderConfig.Voltage.SupplyVoltageTimeConstant = ExtenderConstants.SUPPLY_VOLTAGE_TIME;

    extenderMotor.getConfigurator().apply(extenderConfig);

    // Status signals

    extenderAppliedVoltage = extenderMotor.getMotorVoltage();
    extenderAngle = extenderMotor.getPosition();
    extenderCurrent = extenderMotor.getStatorCurrent();
    extenderDeviceTemp = extenderMotor.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, extenderAppliedVoltage, extenderAngle);
    BaseStatusSignal.setUpdateFrequencyForAll(50, extenderCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, extenderDeviceTemp);

    extenderMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerAppliedVoltage,
        rollerCurrent,
        rollerDeviceTemp,
        rollerVelocity,
        extenderAngle,
        extenderCurrent,
        extenderAppliedVoltage,
        extenderDeviceTemp);

    inputs.rollersCurrentAmps = rollerCurrent.getValue().in(Units.Amps);
    inputs.rollersTempCelsius = rollerDeviceTemp.getValue().in(Units.Celsius);
    inputs.rollersAppliedOutput = rollerAppliedVoltage.getValue().in(Units.Volts);
    inputs.rollersVelocityRPM = rollerVelocity.getValue().in(Units.RPM);

    inputs.extenderPosition = Rotation2d.fromDegrees(extenderAngle.getValue().in(Units.Degrees));
    inputs.extenderCurrentAmps = extenderCurrent.getValue().in(Units.Amps);
    inputs.extenderAppliedOutput = extenderAppliedVoltage.getValue().in(Units.Volts);
    inputs.extenderTempCelsius = extenderDeviceTemp.getValue().in(Units.Celsius);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVel(AngularVelocity vel) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(vel.in(Units.RotationsPerSecond)));
  }

  @Override
  public void configRoller(double kP, double kI, double kV, double maxAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();
    var rollerConfig = rollerMotor.getConfigurator();

    rollerConfig.refresh(pidConfig);
    rollerConfig.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = 1000;

    rollerConfig.apply(pidConfig);
    rollerConfig.apply(mmConfig);
  }

  @Override
  public void setExtenderPos(Rotation2d pos) {
    extenderClosedLoopControl.withPosition(pos.getRotations());
    extenderMotor.setControl(extenderClosedLoopControl);
  }

  @Override
  public void configExtender(double kP, double kD, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    extenderMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    extenderMotor.getConfigurator().apply(slot0Configs);
    extenderMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void zeroMotors() {
    extenderMotor.setPosition(0);
  }

  @Override
  public void setExtenderVoltage(double volts) {
    extenderMotor.setControl(extenderOpenLoopControl.withOutput(volts));
  }

  @Override
  public boolean setExtenderNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = extenderMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    extenderMotor.getConfigurator().apply(config);
    return true;
  }
}
