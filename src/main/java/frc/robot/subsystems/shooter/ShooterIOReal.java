package frc.robot.subsystems.shooter;

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
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.RotationConstants;
import frc.robot.Constants.ShooterConstants.WheelConstants;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX wheelMotor = new TalonFX(Constants.CanIDs.SHOOTER_WHEEL_CAN_ID);
  private final TalonFX hoodMotor = new TalonFX(Constants.CanIDs.SHOOTER_HOOD_CAN_ID);
  private final TalonFX turretMotor = new TalonFX(Constants.CanIDs.SHOOTER_ROTATION_CAN_ID);

  private final StatusSignal<Current> wheelCurrent;
  private final StatusSignal<Temperature> wheelDeviceTemp;
  private final StatusSignal<Voltage> wheelAppliedVoltage;
  private final StatusSignal<AngularVelocity> wheelVelocity;
  private final StatusSignal<Current> turretCurrent;
  private final StatusSignal<Temperature> turretDeviceTemp;
  private final StatusSignal<Voltage> turretAppliedVoltage;
  private final StatusSignal<Angle> turretAngle;
  private final StatusSignal<Temperature> hoodDeviceTemp;
  private final StatusSignal<Voltage> hoodAppliedVoltage;
  private final StatusSignal<Angle> hoodAngle;
  private final StatusSignal<Current> hoodCurrent;

  private final VoltageOut wheelOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut hoodOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut rotationOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage wheelClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage rotationClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage hoodClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  public ShooterIOReal() {
    // Motor config
    TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs wheelCurrentLimitConfig = new CurrentLimitsConfigs();

    wheelCurrentLimitConfig.SupplyCurrentLimit = WheelConstants.SUPPLY_CURRENT_LIMIT;
    wheelCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    wheelCurrentLimitConfig.StatorCurrentLimit = WheelConstants.STATOR_CURRENT_LIMIT;
    wheelCurrentLimitConfig.StatorCurrentLimitEnable = true;
    wheelConfig.CurrentLimits = wheelCurrentLimitConfig;

    wheelConfig.Feedback.SensorToMechanismRatio = WheelConstants.WHEEL_GEARING;
    wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wheelConfig.Voltage.SupplyVoltageTimeConstant = WheelConstants.SUPPLY_VOLTAGE_TIME;

    wheelMotor.getConfigurator().apply(wheelConfig);

    // Status signals

    wheelCurrent = wheelMotor.getStatorCurrent();
    wheelDeviceTemp = wheelMotor.getDeviceTemp();
    wheelAppliedVoltage = wheelMotor.getMotorVoltage();
    wheelVelocity = wheelMotor.getVelocity();

    // Update status signals

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.MAX_HOOD_ANGLE.getRotations();
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.MIN_HOOD_ANGLE.getRotations();

    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
    hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    hoodConfig.Voltage.SupplyVoltageTimeConstant = HoodConstants.SUPPLY_VOLTAGE_TIME;

    hoodMotor.getConfigurator().apply(hoodConfig);

    turretConfig.CurrentLimits.SupplyCurrentLimit = RotationConstants.SUPPLY_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = RotationConstants.STATOR_CURRENT_LIMIT;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        RotationConstants.MAX_ROTATION_ANGLE.getRotations();
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        RotationConstants.MIN_ROTATION_ANGLE.getRotations();

    turretConfig.Feedback.SensorToMechanismRatio = RotationConstants.GEAR_RATIO;
    turretConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    turretConfig.Voltage.SupplyVoltageTimeConstant = HoodConstants.SUPPLY_VOLTAGE_TIME;

    turretMotor.getConfigurator().apply(turretConfig);

    // Status signals

    hoodAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodAngle = hoodMotor.getPosition();
    hoodCurrent = hoodMotor.getStatorCurrent();
    hoodDeviceTemp = hoodMotor.getDeviceTemp();

    turretAppliedVoltage = turretMotor.getMotorVoltage();
    turretAngle = turretMotor.getPosition();
    turretCurrent = turretMotor.getStatorCurrent();
    turretDeviceTemp = turretMotor.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, hoodAppliedVoltage, hoodAngle);
    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, hoodDeviceTemp);
    BaseStatusSignal.setUpdateFrequencyForAll(50, wheelAppliedVoltage, wheelCurrent, wheelVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, wheelDeviceTemp);
    BaseStatusSignal.setUpdateFrequencyForAll(100, turretAppliedVoltage, turretAngle);
    BaseStatusSignal.setUpdateFrequencyForAll(50, turretCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, turretDeviceTemp);

    hoodMotor.optimizeBusUtilization();
    wheelMotor.optimizeBusUtilization();
    turretMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    BaseStatusSignal.refreshAll(
        wheelAppliedVoltage,
        wheelCurrent,
        wheelDeviceTemp,
        wheelVelocity,
        hoodAngle,
        hoodCurrent,
        hoodAppliedVoltage,
        hoodDeviceTemp,
        turretAngle,
        turretCurrent,
        turretAppliedVoltage,
        turretDeviceTemp);

    inputs.wheelCurrentAmps = wheelCurrent.getValue().in(Units.Amps);
    inputs.wheelTempCelsius = wheelDeviceTemp.getValue().in(Units.Celsius);
    inputs.wheelAppliedOutput = wheelAppliedVoltage.getValue().in(Units.Volts);
    inputs.wheelsVelocityRPM = wheelVelocity.getValue().in(Units.RPM);

    inputs.rotationPosition = Rotation2d.fromDegrees(turretAngle.getValue().in(Units.Degrees));
    inputs.rotationCurrentAmps = turretCurrent.getValue().in(Units.Amps);
    inputs.rotationAppliedOutput = turretAppliedVoltage.getValue().in(Units.Volts);
    inputs.rotationTempCelsius = turretDeviceTemp.getValue().in(Units.Celsius);

    inputs.hoodPosition = Rotation2d.fromRotations(hoodAngle.getValue().in(Units.Rotations));
    inputs.hoodCurrentAmps = hoodCurrent.getValue().in(Units.Amps);
    inputs.hoodAppliedOutput = hoodAppliedVoltage.getValue().in(Units.Volts);
    inputs.hoodTempCelsius = hoodDeviceTemp.getValue().in(Units.Celsius);
  }

  @Override
  public void setWheelVoltage(double volts) {
    wheelMotor.setControl(wheelOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setWheelVel(AngularVelocity vel) {
    wheelMotor.setControl(wheelClosedLoopControl.withVelocity(vel.in(Units.RotationsPerSecond)));
  }

  @Override
  public void zeroMotors() {
    turretMotor.setPosition(0.5);
    hoodMotor.setPosition(0);
  }

  @Override
  public void configWheel(double kV, double kP, double kI, double maxAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var wheelConfig = wheelMotor.getConfigurator();

    wheelConfig.refresh(pidConfig);
    wheelConfig.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kI = kI;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = maxAcceleration;

    wheelConfig.apply(pidConfig);
    wheelConfig.apply(mmConfig);
  }

  @Override
  public void setHoodElevation(Rotation2d angle) {
    hoodClosedLoopControl.withPosition(angle.getRotations());
    hoodMotor.setControl(hoodClosedLoopControl);
  }

  @Override
  public void setTurretRotation(Rotation2d angle) {
    double posRotations = angle.getRotations();
    if (posRotations < 0) posRotations += 1;
    rotationClosedLoopControl.withPosition(posRotations);
    turretMotor.setControl(rotationClosedLoopControl);
  }

  @Override
  public void configHood(double kP, double kI, double kD, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    hoodMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;

    hoodMotor.getConfigurator().apply(slot0Configs);
    hoodMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void configRotation(double kP, double kD, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    turretMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    turretMotor.getConfigurator().apply(slot0Configs);
    turretMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodMotor.setControl(hoodOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRotationVoltage(double volts) {
    turretMotor.setControl(rotationOpenLoopControl.withOutput(volts));
  }

  @Override
  public boolean setHoodNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = hoodMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    hoodMotor.getConfigurator().apply(config);
    return true;
  }

  @Override
  public boolean setRotationNeutralMode(NeutralModeValue value) {

    var config = new MotorOutputConfigs();

    var status = turretMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    turretMotor.getConfigurator().apply(config);
    return true;
  }
}
