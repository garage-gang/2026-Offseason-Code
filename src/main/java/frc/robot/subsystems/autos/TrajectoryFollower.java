package frc.robot.subsystems.autos;

import choreo.auto.AutoFactory;

import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.DriveCommands;
import static frc.robot.subsystems.drive.DriveConstants.*;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TrajectoryFollower extends SubsystemBase {

	private final PIDController xController = new PIDController(DRIVE_KP,0,DRIVE_KD);
    private final PIDController yController = new PIDController(DRIVE_KP, 0.0,DRIVE_KD);
    private final PIDController headingController = new PIDController(turnKp, 0, turnKd);
    
	// Trajectory follower for Choreo
	public void followTrajectory(Drive drive, SwerveSample sample) {
		// get pose
		Pose2d pose = drive.getPose();

		// create velocities for robot
		ChassisSpeeds velocities = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
		);
		
		//var temp = velocities.vxMetersPerSecond;
		DriveCommands.autoDrive(drive, velocities);
	}

}
