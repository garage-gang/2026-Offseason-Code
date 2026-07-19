// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Geometry;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrimeShootCommand extends Command {
  /** Creates a new ShooterControl2. */
  private static final LoggedTunableNumber wheelVelocityConfig =
      new LoggedTunableNumber("PrimeShootCommand/Wheel/Velocity", 3500);

  private static final LoggedTunableNumber rotationVelocityConfig =
      new LoggedTunableNumber("PrimeShootCommand/Rotation/Angle", 90);
  private static final LoggedTunableNumber elevationAngleConfig =
      new LoggedTunableNumber("PrimeShootCommand/Elevation/Position", 5);
  private static final LoggedTunableNumber wheelToleranceConfig =
      new LoggedTunableNumber("PrimeShootCommand/Wheel/Tolerance", 100);
  private static final LoggedTunableNumber elevationToleranceConfig =
      new LoggedTunableNumber("PrimeShootCommand/Elevation/Tolerance", 5);
  private static final LoggedTunableNumber turretRotationToleranceConfig =
      new LoggedTunableNumber("PrimeShootCommand/Turret/Tolerance", 10);
  private Shooter shooter;
  private Drive drive;
  private Intake intake;
  private ShootingType shootingType;
  private boolean isPrimed = false;

  public PrimeShootCommand(Shooter shooter, Drive drive, Intake intake, ShootingType shootingType) {
    this.shooter = shooter;
    this.drive = drive;
    this.intake = intake;
    this.shootingType = shootingType;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d targetTurretRotation;
    double targetFlywheelSpeed;
    Rotation2d targetElevationAngle;
    switch (shootingType) {
      case SIMPLE_SHOOT:
        targetTurretRotation = Rotation2d.fromDegrees(5.0);
        targetFlywheelSpeed = 4000.0;
        targetElevationAngle = Rotation2d.fromDegrees(0.0);
        break;
      case HUB_SHOOT:
        targetTurretRotation = updateTurretRotation();
        targetFlywheelSpeed =
            Constants.ShooterConstants.FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.get(turretHubDistance());
        targetElevationAngle =
            Rotation2d.fromDegrees(
                Constants.ShooterConstants.HOOD_HUB_DISTANCE_ANGLE_TABLE.get(turretHubDistance()));
        break;
      case ALLIANCE_SHOOT:
        targetTurretRotation =
            Rotation2d.fromDegrees(DriverStation.getAlliance().get().equals(Alliance.Red) ? 0 : 180)
                .minus(drive.getRotation());
        targetFlywheelSpeed = 5000.0;
        targetElevationAngle = Rotation2d.fromDegrees(45.0);
        break;
      case TUNING_SHOOT:
        targetTurretRotation = updateTurretRotation();
        targetFlywheelSpeed = wheelVelocityConfig.get();
        targetElevationAngle = Rotation2d.fromDegrees(elevationAngleConfig.get());
        turretHubDistance();
        break;
      case TRENCH_SHOT:
        targetTurretRotation = Rotation2d.fromDegrees(rotationVelocityConfig.get());
        targetFlywheelSpeed = wheelVelocityConfig.get();
        targetElevationAngle = Rotation2d.fromDegrees(elevationAngleConfig.get());
        turretHubDistance();
        break;
      default:
        targetTurretRotation = Rotation2d.fromDegrees(5.0);
        targetFlywheelSpeed = 3500.0;
        targetElevationAngle = Rotation2d.fromDegrees(0.0);
    }
    shooter.setRotationPos(targetTurretRotation);
    Logger.recordOutput("RotationTargetPosition", targetTurretRotation);
    shooter.setWheelVel(Units.RPM.of(targetFlywheelSpeed));
    shooter.setHoodElevation(targetElevationAngle);

    boolean elevationInPosition =
        withinTolerance(
            targetElevationAngle.getDegrees(),
            shooter.getHoodElevation().getDegrees(),
            elevationToleranceConfig.get());
    boolean turretInPosition =
        withinTolerance(
            targetTurretRotation.getDegrees(),
            Rotation2d.fromRadians(
                    (MathUtil.angleModulus(shooter.getTurretRotation().getRadians())))
                .getDegrees(),
            turretRotationToleranceConfig.get());
    boolean wheelAtVelocity =
        withinTolerance(
            targetFlywheelSpeed, shooter.getWheelVel().in(Units.RPM), wheelToleranceConfig.get());
    boolean checkExtenderPosition =
        Constants.IntakeConstants.ExtenderConstants.MIN_REQ_EXTENDER_ANGLE.getDegrees()
            < intake.getExtenderPos().getDegrees();

    final var msg = "target=%s real=%s tolerance=%s okay=%s";
    SmartDashboard.putString(
        "hood elevation",
        msg.formatted(
            targetElevationAngle.getDegrees(),
            shooter.getHoodElevation().getDegrees(),
            elevationToleranceConfig.get(),
            elevationInPosition));
    SmartDashboard.putString(
        "turret rotation",
        msg.formatted(
            targetTurretRotation.getDegrees(),
            shooter.getTurretRotation().getDegrees(),
            turretRotationToleranceConfig.get(),
            turretInPosition));
    SmartDashboard.putString(
        "wheel velocity",
        msg.formatted(
            targetFlywheelSpeed,
            shooter.getWheelVel().in(Units.RPM),
            wheelToleranceConfig.get(),
            wheelAtVelocity));
    SmartDashboard.putString(
        "extender",
        "%s < %s : %s"
            .formatted(
                Constants.IntakeConstants.ExtenderConstants.MIN_REQ_EXTENDER_ANGLE.getDegrees(),
                intake.getExtenderPos().getDegrees(),
                checkExtenderPosition));

    isPrimed = elevationInPosition && turretInPosition && wheelAtVelocity && checkExtenderPosition;

    String PrimeShootString = "PrimeShootCommand";

    Logger.recordOutput(PrimeShootString + "/hoodInPosition", elevationInPosition);
    Logger.recordOutput(PrimeShootString + "/rotationInPosition", turretInPosition);
    Logger.recordOutput(PrimeShootString + "/wheelAtPosition", wheelAtVelocity);
    Logger.recordOutput(PrimeShootString + "/checkExtenderPosition", checkExtenderPosition);
    Logger.recordOutput(
        PrimeShootString + "/actualTurretPosition", shooter.getTurretRotation().getDegrees());
  }

  private Rotation2d updateTurretRotation() {
    final Rotation2d heading =
        Geometry.headingPosition(adjustTurretPosition(), FieldConstants.ourHubPosition());
    double targetRotationDegrees =
        heading.getDegrees() - drive.getRotation().getDegrees() - fudgeFactor();

    Rotation2d targetRotationPos =
        Rotation2d.fromRadians(
            (MathUtil.angleModulus(Rotation2d.fromDegrees(targetRotationDegrees).getRadians())));
    return targetRotationPos;
  }

  private double fudgeFactor() {
    return drive.getPose().getY() > Constants.FieldConstants.FIELD_WIDTH / 2 ? -6 : 6;
  }

  private Translation2d adjustTurretPosition() {
    ChassisSpeeds robotVelocityChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    Translation2d robotVelocity =
        new Translation2d(
            robotVelocityChassisSpeeds.vxMetersPerSecond,
            robotVelocityChassisSpeeds.vyMetersPerSecond);
    Translation2d adjustTurretPosition =
        turretFieldPosition()
            .plus(robotVelocity.times(ShooterConstants.RotationConstants.TIME_ADJUST_TURRET));
    Logger.recordOutput("PrimeShootCommand/AdjustedTurretPosition", adjustTurretPosition);
    return adjustTurretPosition;
  }

  private boolean withinTolerance(double targetState, double currentState, double tolerance) {
    return Math.abs(currentState - targetState) < tolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setWheelVel((Units.RPM.of(0)));
    shooter.setHoodElevation(new Rotation2d(0));
    isPrimed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isPrimed() {
    return isPrimed;
  }

  @AutoLogOutput(key = "PrimeShootCommand/TurretPosition")
  private Translation2d turretFieldPosition() {
    Translation2d turretTranslation2d =
        Constants.ShooterConstants.TURRET_TRANSLATION.rotateBy(drive.getRotation());
    Translation2d turretFieldPosition = turretTranslation2d.plus(drive.getPose().getTranslation());
    return turretFieldPosition;
  }

  private double turretHubDistance() {
    double turretHubDistance =
        Constants.FieldConstants.ourHubPosition().getDistance(adjustTurretPosition());
    Logger.recordOutput("PrimeShootCommand/HubDistance", turretHubDistance);
    return turretHubDistance;
  }

  public enum ShootingType {
    SIMPLE_SHOOT,
    HUB_SHOOT,
    ALLIANCE_SHOOT,
    TUNING_SHOOT,
    TRENCH_SHOT
  };
}
