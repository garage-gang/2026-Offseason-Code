package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.ExtenderConstants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private static final LoggedTunableNumber rKP = new LoggedTunableNumber("Intake/Roller/kP");
  private static final LoggedTunableNumber rKI = new LoggedTunableNumber("Intake/Roller/kI");
  private static final LoggedTunableNumber rKD = new LoggedTunableNumber("Intake/Roller/kD");
  private static final LoggedTunableNumber rKV = new LoggedTunableNumber("Intake/Roller/kV");
  private static final LoggedTunableNumber eKP = new LoggedTunableNumber("Intake/Extender/kP");
  private static final LoggedTunableNumber eKD = new LoggedTunableNumber("Intake/Extender/kD");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig =
      new LoggedTunableNumber("Intake/Roller/Acceleration");
  private static final LoggedTunableNumber extenderMaxVelocityConfig =
      new LoggedTunableNumber("Intake/Extender/MaxVel");
  private static final LoggedTunableNumber extenderTargetAccelerationConfig =
      new LoggedTunableNumber("Intake/Extender/TargetAcceleration");

  static {
    if (Constants.currentMode == Mode.REAL) {
      rKP.initDefault(0.0025);
      rKI.initDefault(0.15);
      rKV.initDefault(0.15);
      rollerTargetAccelerationConfig.initDefault(300.0);

      eKP.initDefault(100);
      eKD.initDefault(0);

      extenderMaxVelocityConfig.initDefault(10);
      extenderTargetAccelerationConfig.initDefault(10);
    } else {
      rKP.initDefault(0.0025);
      rKI.initDefault(0.15);
      rKV.initDefault(0.15);
      rollerTargetAccelerationConfig.initDefault(1000.0);

      eKP.initDefault(2);
      eKD.initDefault(1);

      extenderMaxVelocityConfig.initDefault(40);
      extenderTargetAccelerationConfig.initDefault(80);
    }
  }

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    configExtender();
    configRoller();
  }

  public void periodic() {
    Logger.processInputs("Intake", inputs);
    int hc = hashCode();
    if (eKP.hasChanged(hc)
        || eKD.hasChanged(hc)
        || extenderMaxVelocityConfig.hasChanged(hc)
        || extenderTargetAccelerationConfig.hasChanged(hc)) configExtender();
    if (rKP.hasChanged(hc)
        || rKD.hasChanged(hc)
        || rKV.hasChanged(hc)
        || rollerTargetAccelerationConfig.hasChanged(hc)) configRoller();
    intakeIO.updateInputs(inputs);
  }

  private void configExtender() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = extenderTargetAccelerationConfig.get();
    mmConfigs.MotionMagicCruiseVelocity = extenderMaxVelocityConfig.get();
    intakeIO.configExtender(eKP.get(), eKD.get(), mmConfigs);
  }

  private void configRoller() {
    MotionMagicConfigs rollermmConfigs = new MotionMagicConfigs();
    rollermmConfigs.MotionMagicAcceleration = rollerTargetAccelerationConfig.get();
    intakeIO.configRoller(rKP.get(), rKI.get(), rKV.get(), rollerTargetAccelerationConfig.get());
  }

  public void setRollerVoltage(double volts) {
    intakeIO.setRollerVoltage(volts);
  }

  public void setExtenderVoltage(double volts) {
    intakeIO.setExtenderVoltage(volts);
  }

  public void setRollerVel(AngularVelocity vel) {
    intakeIO.setRollerVel(vel);
  }

  public void setExtenderPos(Rotation2d pos) {
    intakeIO.setExtenderPos(pos);
  }

  public Rotation2d getExtenderPos() {
    return inputs.extenderPosition;
  }

  // Returns true if extender is out at least 90 degrees from zero, returns false if else
  public boolean checkExtenderPosition() {
    return getExtenderPos().getDegrees()
        > Constants.IntakeConstants.ExtenderConstants.MIN_REQ_EXTENDER_ANGLE.getDegrees();
  }

  public void zeroMotors() {
    intakeIO.zeroMotors();
  }

  public void extendExtender() {
    setExtenderPos(ExtenderConstants.MAX_EXTENDER_ANGLE);
  }

  public void retractExtender() {
    setExtenderPos(Rotation2d.kZero);
  }

  public void releaseFuel() {
    setRollerVoltage(12.0);
  }

  public void intakeFuel() {
    setRollerVoltage(-12.0);
  }

  public void stopRoller() {
    setRollerVel(Units.RPM.of(0));
  }
}
