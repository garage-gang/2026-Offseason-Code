// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  // TODO: Update names in PhotonVision to a more logical naming scheme. See below names.
  public static String backElectronicsCamera = "backElectronicsSide";
  public static String backTurretCamera = "backTurretSide";
  public static String backCenterCamera = "backCenter";

  // STUDENT TODO: FIX THE FORMATTING PLEASE FOR THE LOVE OF MY SANITY -- Drew.
  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  // New camera transforms updated 07/18/2026
  // Order of signs: (-,+,+, (+,+,+))
  public static Transform3d backElectronicsCameraToRobot =
      new Transform3d(-0.2936, 0.3084, 0.2145, new Rotation3d(0.0, 0.0, Math.PI / 2));
  // Order of signs: (-, -, +, (0,0,-))
  public static Transform3d backTurretCameraToRobot =
      new Transform3d(-0.2936, -0.3084, 0.2145, new Rotation3d(0.0, 0.0, (3 * Math.PI) / 2));
  // Order of signs: (-,+,+, (0,+,0))
  public static Transform3d backCenterToRobot =
      new Transform3d(-0.1773428, 0.1070864, 0.4154043, new Rotation3d(0.0, -0.4636, 3.1415));

  // The below transform does not seem to be part of the vision transforms for estimating robot
  // position.
  // Moving it down a few lines for organization.
  public static Transform3d tagToGoal =
      new Transform3d(0, 2.5, 0.0, new Rotation3d(0.0, -0.2, 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.225;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0; // Meters
  public static double angularStdDevBaseline = 0; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.004, // Camera 0, backElectronicsSide
        0.004, // Camera 1, backTurretSide
        0.1474 // Camera 2, backCenter
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = 0; // No rotation data available
}
