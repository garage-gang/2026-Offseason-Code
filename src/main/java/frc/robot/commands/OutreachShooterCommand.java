// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.units.Units;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutreachShooterCommand extends Command {
	/** Creates a new outreachShooterCommand. */
	private Shooter shooter;
	private DoubleSupplier xSupplier;
	private DoubleSupplier ySupplier;

	public OutreachShooterCommand(Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.shooter = shooter;
		addRequirements(shooter);
		this.ySupplier = ySupplier;
		this.xSupplier = xSupplier;

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		shooter.setRotationVoltage(xSupplier.getAsDouble() * -6);
		shooter.setWheelVel(Units.RPM.of(2000));
		shooter.setHoodVoltage(ySupplier.getAsDouble() * -4);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
