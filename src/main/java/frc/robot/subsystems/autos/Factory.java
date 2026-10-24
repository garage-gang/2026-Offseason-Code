package frc.robot.subsystems.autos;

import choreo.auto.AutoFactory;

import frc.robot.subsystems.drive.Drive;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Factory extends SubsystemBase {

	private final AutoFactory autoFactory;

	public Factory(Drive drive) {
		autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
	}

	public Command babysFirstAuto() {
		Command babyTrajectory = autoFactory.trajectoryCmd("babyPath");

		return Commands.sequence(autoFactory.resetOdometry("babyPath"), babyTrajectory);
	}

}
