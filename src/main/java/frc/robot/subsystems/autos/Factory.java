package frc.robot.subsystems.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.PrimeShootCommand;
import frc.robot.commands.ShakeIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ExtendAndIntakeCommand;
import frc.robot.commands.PrimeShootCommand.ShootingType;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Factory extends SubsystemBase {

	private final AutoFactory autoFactory;
	private final Shooter shooter;
	private final Intake intake;
	private final Drive drive;
	private final Elevator elevator;

	public Factory(Drive drive, Shooter shooter, Intake intake, Elevator elevator) {
		autoFactory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
		this.shooter = shooter;
		this.intake = intake;
		this.drive = drive;
		this.elevator = elevator;
	}

	public Command babysFirstAuto() {
		Command babyTrajectory = autoFactory.trajectoryCmd("babyPath");

		return Commands.sequence(autoFactory.resetOdometry("babyPath"), babyTrajectory);

	}

	public Command middleShootAuto() {
		PrimeShootCommand primeShootCommand = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);
		Command middleShoot = autoFactory.trajectoryCmd("midShoot");

		return Commands.sequence(autoFactory.resetOdometry("midShoot"), middleShoot, primeShootCommand);
	}

	public Command leftTrenchShootAuto() {

		PrimeShootCommand primeShootCommand = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);
		ExtendAndIntakeCommand extendAndIntakeCommand = new ExtendAndIntakeCommand(intake);
		ShakeIntakeCommand shakeIntakeCommand = new ShakeIntakeCommand(intake);

		Command pathToo = autoFactory.trajectoryCmd("PathTooTS");
		Command pathFrom = autoFactory.trajectoryCmd("PathFromTS");

		return Commands.sequence(autoFactory.resetOdometry("PathTooTS"),
		        Commands.deadline(pathToo, extendAndIntakeCommand), pathFrom,
		        Commands.parallel(primeShootCommand, shakeIntakeCommand));
	}

	public Command rightTrenchShootAuto() {
		AutoRoutine trenchShootAutoRoutine = autoFactory.newRoutine("TrenchShoot");

		AutoTrajectory pathToo = trenchShootAutoRoutine.trajectory("pathTooTS").mirrorY();
		AutoTrajectory pathFrom = trenchShootAutoRoutine.trajectory("pathFromTS").mirrorY();

		return Commands.sequence(pathToo.resetOdometry(), pathToo.cmd(), pathFrom.cmd());
	}

	public Command middleShootChor() {
		Command midTraj = autoFactory.trajectoryCmd("BasicShootC");

		PrimeShootCommand primeShootCommand = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);
		PrimeShootCommand simplePrimeShootCommand = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.SIMPLE_SHOOT);
		PrimeShootCommand alliancePrimeShootCommand = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.ALLIANCE_SHOOT);
		ShootCommand shootCommand = new ShootCommand(elevator, primeShootCommand, simplePrimeShootCommand,
		        alliancePrimeShootCommand);

		return Commands.sequence(autoFactory.resetOdometry("BasicShootC"), midTraj, primeShootCommand, shootCommand);

	}

}
