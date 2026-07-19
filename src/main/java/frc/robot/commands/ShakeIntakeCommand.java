package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;

public class ShakeIntakeCommand extends Command {

  private Intake intake;

  public ShakeIntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setExtenderVoltage(-1);
    Commands.waitSeconds(0.5);
    intake.setExtenderVoltage(1);
    Commands.waitSeconds(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setExtenderVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
