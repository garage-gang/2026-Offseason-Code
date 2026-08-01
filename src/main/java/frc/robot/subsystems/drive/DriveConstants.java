// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
	public static final double MAX_SPEED_METERS_PER_SEC = 4.46;
	public static final double ODOMETRY_FREQUENCY = 100.0; // Hz
	public static final double TRACK_WIDTH = Units.inchesToMeters(22.0);
	public static final double WHEEL_BASE = Units.inchesToMeters(22.0);
	public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
	public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
	        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
	        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
	        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
	        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0)};

	public static final Rotation2d FRONT_LEFT_ZERO_ROTATION = new Rotation2d(0 + 3.14 + 1.57);
	public static final Rotation2d FRONT_RIGHT_ZERO_ROTATION = new Rotation2d(0);
	public static final Rotation2d BACK_LEFT_ZERO_ROTATION = new Rotation2d(3.14159);
	public static final Rotation2d BACK_RIGHT_ZERO_ROTATION = new Rotation2d(1.57079);

	public static final int PIGEON_CAN_ID = 9;

	public static final int FRONT_LEFT_DRIVE_CAN_ID = 8;
	public static final int BACK_LEFT_DRIVE_CAN_ID = 4;
	public static final int FRONT_RIGHT_DRIVE_CAN_ID = 2;
	public static final int BACK_RIGHT_DRIVE_CAN_ID = 6;

	public static final int FRONT_LEFT_TURN_CAN_ID = 7;
	public static final int BACK_LEFT_TURN_CAN_ID = 3;
	public static final int FRONT_RIGHT_TURN_CAN_ID = 1;
	public static final int BACK_RIGHT_TURN_CAN_ID = 5;

	// Drive motor configuration
	public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
	public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
	public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (13 * 15);
	// MAXSwerve with 13 pinion teeth
	// and 22 spur teeth
	public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

	// Drive encoder configuration
	public static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_MOTOR_REDUCTION; // Rotor Rotations
	                                                                                                // ->
	// Wheel Radians
	public static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION; // Rotor
	                                                                                                         // RPM ->
	// Wheel Rad/Sec

	// Drive PID configuration
	public static final double DRIVE_KP = 0.0;
	public static final double DRIVE_KD = 0.0;
	public static final double DRIVE_KS = 0.0;
	public static final double DRIVE_KV = 0.1;
	public static final double DRIVE_SIM_P = 0.05;
	public static final double DRIVE_SIM_D = 0.0;
	public static final double DRIVE_SIM_KS = 0.0;
	public static final double DRIVE_SIM_KV = 0.0789;

	// Turn motor configuration
	public static final boolean turnInverted = false;
	public static final int turnMotorCurrentLimit = 20;
	public static final double turnMotorReduction = 9424.0 / 203.0;
	public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

	// Turn encoder configuration
	public static final boolean turnEncoderInverted = true;
	public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
	public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

	// Turn PID configuration
	public static final double turnKp = 2.0;
	public static final double turnKd = 0.0;
	public static final double turnSimP = 8.0;
	public static final double turnSimD = 0.0;
	public static final double turnPIDMinInput = 0; // Radians
	public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

	// PathPlanner configuration
	public static final double robotMassKg = 74.088;
	public static final double robotMOI = 6.883;
	public static final double wheelCOF = 1.2;
	public static final RobotConfig ppConfig = new RobotConfig(robotMassKg, robotMOI,
	        new ModuleConfig(WHEEL_RADIUS_METERS, MAX_SPEED_METERS_PER_SEC, wheelCOF,
	                DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION), DRIVE_MOTOR_CURRENT_LIMIT, 1),
	        MODULE_TRANSLATIONS);

	// Robot Crawl max speeds
	// 0.05 = lowest possible speed after upping the value 
	public static final double robotCrawlModifier = 0.05;
}
