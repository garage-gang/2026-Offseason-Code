package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class Geometry {

  public static Rotation2d headingPosition(Translation2d pose1, Translation2d pose2) {

    double xDistance = pose2.getX() - pose1.getX();
    double YDistance = pose2.getY() - pose1.getY();
    Rotation2d position = Rotation2d.fromRadians(Math.atan2(YDistance, xDistance));
    return position;
  }

  public static Translation2d flippedPosition(Translation2d pose) {

    Translation2d flippedPosition =
        new Translation2d(FieldConstants.FIELD_LENGTH - pose.getX(), pose.getY());
    return flippedPosition;
  }
}
