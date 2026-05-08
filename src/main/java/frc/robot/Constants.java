// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class IntakeConstants {
    public class ExtenderConstants {

      public static final double GEAR_RATIO = 45; // will need to be changed ALOT
      public static final Rotation2d MAX_EXTENDER_ANGLE = Rotation2d.fromDegrees(116.015625);
      public static final Rotation2d MIN_EXTENDER_ANGLE = Rotation2d.fromDegrees(0);
      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
      public static final double EXTENDER_MOI = 0.1;
      public static final Distance EXTENDER_LENGTH = Units.Inches.of(12.5);
      public static final Rotation2d MIN_REQ_EXTENDER_ANGLE = Rotation2d.fromDegrees(8);
    }

    public class RollerConstants {

      public static final double SUPPLY_CURRENT_LIMIT = 30;
      public static final double STATOR_CURRENT_LIMIT = 30;
      public static final double ROLLER_GEARING = 3;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
      public static final double ROLLER_MOI = 0.1;
    }
  }

  public class FieldConstants {
    // set the hub position and alliance position translation2d,
    // used that to translate the value and get
    // our hub position
    // depending on aliance color.

    public static final Translation2d BLUE_HUB_POSITION =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(18).get().getX(),
            VisionConstants.aprilTagLayout.getTagPose(26).get().getY());

    public static final Translation2d RED_HUB_POSITION =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(4).get().getX(),
            VisionConstants.aprilTagLayout.getTagPose(4).get().getY());

    public static final Translation2d BLUE_ALLIANCE_BOTTOM_POSITION =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(22).get().getX() - 0.5,
            VisionConstants.aprilTagLayout.getTagPose(22).get().getY() + 0.5);

    public static final Translation2d BLUE_ALLIANCE_TOP_POSITION =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(17).get().getX() - 0.5,
            VisionConstants.aprilTagLayout.getTagPose(17).get().getY() - 0.5);

    public static final Translation2d RED_ALLIANCE_BOTTOM_POSITION =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(1).get().getX() + 0.5,
            VisionConstants.aprilTagLayout.getTagPose(1).get().getY() + 0.5);

    public static final Translation2d RED_ALLIANCE_TOP_POSITION =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(6).get().getX() + 0.5,
            VisionConstants.aprilTagLayout.getTagPose(6).get().getY() - 0.5);

    public static final double FIELD_LENGTH = 16.54048;
    public static final double FIELD_WIDTH = 8.06958;

    public static Translation2d ourHubPosition() {
      Translation2d hubP = new Translation2d();
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        hubP = BLUE_HUB_POSITION;
      } else if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        hubP = RED_HUB_POSITION;
      }
      return hubP;
    }

    public static Translation2d targetedTrenchDirection(Pose2d robotPose) {
      Translation2d trenchTurretDecision = new Translation2d();
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        if (robotPose.getY() > FIELD_WIDTH / 2.0) {
          trenchTurretDecision = BLUE_ALLIANCE_BOTTOM_POSITION;
        } else trenchTurretDecision = BLUE_ALLIANCE_TOP_POSITION;
      } else {

        if (robotPose.getY() > FIELD_WIDTH / 2.0) {
          trenchTurretDecision = RED_ALLIANCE_BOTTOM_POSITION;
        } else trenchTurretDecision = RED_ALLIANCE_TOP_POSITION;
      }
      return trenchTurretDecision;
    }
  }

  public class ElevatorConstants {

    public static final double SUPPLY_CURRENT_LIMIT = 30;
    public static final double STATOR_CURRENT_LIMIT = 30;
    public static final double SUPPLY_VOLTAGE_TIME = 0.02;
    public static final double ELEVATOR_GEARING = 14.0 / 30.0;
    public static final double ELEVATOR_MOI = 0.01;
  }

  public class CanIDs {
    public static final int FRONT_LEFT_WHEEL_CAN_ID = 1;
    public static final int FRONT_LEFT_ANGLE_CAN_ID = 2;
    public static final int FRONT_RIGHT_WHEEL_CAN_ID = 3;
    public static final int FRONT_RIGHT_ANGLE_CAN_ID = 4;
    public static final int BACK_LEFT_WHEEL_CAN_ID = 5;
    public static final int BACK_LEFT_ANGLE_CAN_ID = 6;
    public static final int BACK_RIGHT_WHEEL_CAN_ID = 7;
    public static final int BACK_RIGHT_ANGLE_CAN_ID = 8;
    public static final int INTAKE_EXTENDER_CAN_ID = 16;
    public static final int INTAKE_ROLLER_CAN_ID = 11;
    public static final int ELEVATOR_CAN_ID = 12;
    public static final int SHOOTER_WHEEL_CAN_ID = 13;
    public static final int SHOOTER_HOOD_CAN_ID = 14;
    public static final int SHOOTER_ROTATION_CAN_ID = 15;
  }

  public final class ShooterConstants {
    public static final Translation2d TURRET_TRANSLATION = new Translation2d(-0.130175, -0.1397);

    public final class HoodConstants {
      public static final Rotation2d MIN_HOOD_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d MAX_HOOD_ANGLE = Rotation2d.fromDegrees(40);
      public static final double HOOD_MOI = 0.0001;
      public static final double STATOR_CURRENT_LIMIT = 35;
      public static final double SUPPLY_CURRENT_LIMIT = 35;
      public static final double MAX_VELOCITY = 10;
      public static final double TARGET_ACCELERATION = 10;
      public static final double GEAR_RATIO = 11.3333333333;
      public static final double SUPPLY_VOLTAGE_TIME = 0.5;
      public static final double MAX_VOLTAGE = 12.0;
      public static final double kDefaultPeriod = 0.02;
      public static final Distance HOOD_LENGTH = Units.Meters.of(0.5);
    }

    public final class RotationConstants {
      public static final Rotation2d MIN_ROTATION_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d MAX_ROTATION_ANGLE = Rotation2d.fromDegrees(390);
      public static final double ROTATION_MOI = 0.1;
      public static final double STATOR_CURRENT_LIMIT = 20;
      public static final double SUPPLY_CURRENT_LIMIT = 20;
      public static final double MAX_VELOCITY = 10;
      public static final double TARGET_ACCELERATION = 10;
      public static final double GEAR_RATIO = 13.333333;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
      public static final double TIME_ADJUST_TURRET = 2;
    }

    public final class WheelConstants {
      public static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inches
      public static final double WHEEL_GEARING = 0.88889;
      public static final double WHEEL_MOI = 0.01;
      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
    }

    public static final InterpolatingDoubleTreeMap FLYWHEEL_HUB_DISTANCE_SPEED_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      // FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(2.34, 3500.0);
      // FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(3.218, 4200.0);
      // FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(3.35, 4400.0);
      // FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(4.09, 4600.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(3.89, 4200.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(3.427, 4200.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(5.503, 5000.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(1.994, 3250.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(3.217, 3700.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(4.788, 5000.0);
      FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.put(5.686, 5700.0);
    }

    public static final InterpolatingDoubleTreeMap HOOD_HUB_DISTANCE_ANGLE_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      // HOOD_HUB_DISTANCE_ANGLE_TABLE.put(2.34, 5.0);
      // HOOD_HUB_DISTANCE_ANGLE_TABLE.put(3.218, 20.0);
      // HOOD_HUB_DISTANCE_ANGLE_TABLE.put(3.35, 5.0);
      // HOOD_HUB_DISTANCE_ANGLE_TABLE.put(4.09, 30.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(3.89, 11.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(3.427, 11.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(5.503, 16.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(2.548, 5.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(1.994, 5.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(3.217, 15.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(4.788, 11.0);
      HOOD_HUB_DISTANCE_ANGLE_TABLE.put(5.686, 13.0);
    }

    public static final InterpolatingDoubleTreeMap FLYWHEEL_ALLIANCE_DISTANCE_SPEED_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      FLYWHEEL_ALLIANCE_DISTANCE_SPEED_TABLE.put(0.0, 0.0);
      FLYWHEEL_ALLIANCE_DISTANCE_SPEED_TABLE.put(1.0, 10.0);
      FLYWHEEL_ALLIANCE_DISTANCE_SPEED_TABLE.put(2.0, 30.0);
    }

    public static final InterpolatingDoubleTreeMap HOOD_ALLIANCE_DISTANCE_ANGLE_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      HOOD_ALLIANCE_DISTANCE_ANGLE_TABLE.put(0.0, 0.0);
      HOOD_ALLIANCE_DISTANCE_ANGLE_TABLE.put(1.0, 10.0);
      HOOD_ALLIANCE_DISTANCE_ANGLE_TABLE.put(2.0, 30.0);
    }
  }

  public static final double MAX_VOLTAGE = 12.0;
  public static final double kDefaultPeriod = 0.02;
}
