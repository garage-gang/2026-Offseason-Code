package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.LoggedTunableNumber;

public class ShootCommand extends Command {
  private Elevator elevator;
  private PrimeShootCommand primeShootCommand;
  private PrimeShootCommand simplePrimeShootCommand;
  private PrimeShootCommand alliancePrimeShootCommand;
  private static final LoggedTunableNumber elevatorVoltageConfig =
      new LoggedTunableNumber("Elevator/Elevator/Velocity", 11.5);

  public ShootCommand(
      Elevator elevator,
      PrimeShootCommand primeShootCommand,
      PrimeShootCommand simplePrimeShootCommand,
      PrimeShootCommand alliancePrimeShootCommand) {
    this.elevator = elevator;
    this.primeShootCommand = primeShootCommand;
    this.simplePrimeShootCommand = simplePrimeShootCommand;
    this.alliancePrimeShootCommand = alliancePrimeShootCommand;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    if (primeShootCommand.isPrimed()
        || simplePrimeShootCommand.isPrimed()
        || alliancePrimeShootCommand.isPrimed())
      elevator.setElevatorVoltage(elevatorVoltageConfig.get());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorVoltage(0);
  }
}
