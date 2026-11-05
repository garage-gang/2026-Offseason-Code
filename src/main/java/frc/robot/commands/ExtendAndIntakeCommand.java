package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.Intake.Intake;

public class ExtendAndIntakeCommand extends Command {

	private Intake intake;

	public ExtendAndIntakeCommand(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.setExtenderPos(ExtenderConstants.MAX_EXTENDER_ANGLE);
		intake.intakeFuel();
	}

	@Override
	public void end(boolean interrupted) {
		intake.setExtenderPos(ExtenderConstants.MIN_EXTENDER_ANGLE);
		intake.setRollerVoltage(0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
