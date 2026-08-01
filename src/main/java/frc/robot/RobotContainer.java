// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.backCenterCamera;
import static frc.robot.subsystems.vision.VisionConstants.backCenterToRobot;
import static frc.robot.subsystems.vision.VisionConstants.backElectronicsCamera;
import static frc.robot.subsystems.vision.VisionConstants.backElectronicsCameraToRobot;
import static frc.robot.subsystems.vision.VisionConstants.backTurretCamera;
import static frc.robot.subsystems.vision.VisionConstants.backTurretCameraToRobot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TuningDriveCommands;
import frc.robot.commands.PrimeShootCommand;
import frc.robot.commands.PrimeShootCommand.ShootingType;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Drive drive;
	private final Intake intake;
	private final Elevator elevator;

	private final Shooter shooter;
	private final Vision vision;
	// Controller
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController coDriverController = new CommandXboxController(1);
	private double falseXSignal;
	private double falseYSignal;
	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL :
				// Real robot, instantiate hardware IO implementations
				drive = new Drive(new GyroIOPigeon2(), new ModuleIOSpark(0), new ModuleIOSpark(1), new ModuleIOSpark(2),
				        new ModuleIOSpark(3));

				intake = new Intake(new IntakeIOReal());
				elevator = new Elevator(new ElevatorIOReal());
				shooter = new Shooter(new ShooterIOReal());

				vision = new Vision(drive::addVisionMeasurement,
				        new VisionIOPhotonVision(backElectronicsCamera, backElectronicsCameraToRobot),
				        new VisionIOPhotonVision(backTurretCamera, backTurretCameraToRobot),
				        new VisionIOPhotonVision(backCenterCamera, backCenterToRobot));
				break;

			case SIM :
				// Sim robot, instantiate physics sim IO implementations
				drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(),
				        new ModuleIOSim());
				intake = new Intake(new IntakeIOSim());
				elevator = new Elevator(new ElevatorIOSim());
				shooter = new Shooter(new ShooterIOSim());
				vision = new Vision(drive::addVisionMeasurement,
				        new VisionIOPhotonVisionSim(backElectronicsCamera, backElectronicsCameraToRobot,
				                drive::getPose),
				        new VisionIOPhotonVisionSim(backTurretCamera, backTurretCameraToRobot, drive::getPose),
				        new VisionIOPhotonVisionSim(backCenterCamera, backCenterToRobot, drive::getPose));

				break;

			default :
				// Replayed robot, disable IO implementations
				drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {},
				        new ModuleIO() {});
				intake = new Intake(new IntakeIO() {});
				elevator = new Elevator(new ElevatorIO() {});
				shooter = new Shooter(new ShooterIO() {});
				vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
				break;
		}

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
		// autoChooser.addOption(
		// "Drive Wheel Radius Characterization",
		// DriveCommands.wheelRadiusCharacterization(drive));
		// autoChooser.addOption(
		// "Drive Simple FF Characterization",
		// DriveCommands.feedforwardCharacterization(drive));
		// autoChooser.addOption(
		// "Drive SysId (Quasistatic Forward)",
		// drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		// autoChooser.addOption(
		// "Drive SysId (Quasistatic Reverse)",
		// drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		// autoChooser.addOption(
		// "Drive SysId (Dynamic Forward)",
		// drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		// autoChooser.addOption(
		// "Drive SysId (Dynamic Reverse)",
		// drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		PrimeShootCommand primeShootCommand = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandSecond = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandThird = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandFourth = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandFifth = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandSix = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandSeven = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.HUB_SHOOT);
		PrimeShootCommand primeShootCommandEight = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.HUB_SHOOT);

		PrimeShootCommand autoPrimeShootCommand = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);
		NamedCommands.registerCommand("PrimeShoot",
		        new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT));
		NamedCommands.registerCommand("Shoot",
		        new ShootCommand(elevator, autoPrimeShootCommand, autoPrimeShootCommand, autoPrimeShootCommand));

		try {
			// List paths for auto selection
			// Basic center path for moving back from the hub and then shooting.
			PathPlannerPath centerShoot = PathPlannerPath.fromPathFile("Center_Shoot");
			// Left Side collect fuel from middle and then return to shoot
			// This same path is flipped to create the Right Trench Auto Path.
			PathPlannerPath rightTrenchBumperShootAutoPath = PathPlannerPath.fromPathFile("Left_Side_Collect_Shoot");
			PathPlannerPath leftTrenchBumpShootAutoPath = PathPlannerPath.fromPathFile("Left_Side_Collect_Shoot");
			// Center Depot Shoot for collecting from the depot and then shooting fuel
			PathPlannerPath centerDepotShoot = PathPlannerPath.fromPathFile("Center_Depot_Shoot");

			// ------------------------ Center Hub Shoot Auto
			// --------------------------------------------------------------------
			autoChooser.addOption("Center Hub Shoot",
			        Commands.runOnce(() -> drive.setPose(DriverStation.getAlliance().get().equals(Alliance.Blue)
			                ? centerShoot.getStartingHolonomicPose().get()
			                : centerShoot.flipPath().getStartingHolonomicPose().get())).andThen(
			                        Commands.runOnce(() -> intake.extendExtender())
			                                .alongWith(AutoBuilder.followPath(centerShoot)
			                                        .andThen(primeShootCommandFourth.alongWith(new ShootCommand(
			                                                elevator, primeShootCommandFourth, primeShootCommandFourth,
			                                                primeShootCommandFourth))))));
			// ------------------------------- Center Collect from Depot Shoot Auto
			// -----------------------------------------------------------
			autoChooser.addOption("Center Depot Shoot", Commands.runOnce(() -> {
				var startPose =
				        // drive.setPose(
				        DriverStation.getAlliance().get().equals(Alliance.Blue)
				                ? centerDepotShoot.getPathPoses().get(0)
				                : centerDepotShoot.flipPath().getPathPoses().get(0);
				if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
					startPose = new Pose2d(startPose.getTranslation(),
					        startPose.getRotation().rotateBy(Rotation2d.kPi));
				}
				drive.setPose(startPose);
			}).andThen(
			        Commands.runOnce(() -> intake.extendExtender()).alongWith(Commands.run(() -> intake.intakeFuel()))
			                .alongWith(AutoBuilder.followPath(centerDepotShoot)
			                        .andThen(Commands.waitSeconds(1.0)
			                                .deadlineFor(primeShootCommandSix
			                                        .alongWith(new ShootCommand(elevator, primeShootCommandSix,
			                                                primeShootCommandSix, primeShootCommandSix))
			                                        .alongWith(Commands
			                                                .repeatingSequence(
			                                                        Commands.runOnce(() -> intake.setExtenderPos(
			                                                                Rotation2d.fromDegrees(30))),
			                                                        Commands.waitSeconds(1),
			                                                        Commands.runOnce(() -> intake.setExtenderPos(
			                                                                Rotation2d.fromDegrees(75))),
			                                                        Commands.waitSeconds(1))
			                                                .alongWith(Commands
			                                                        .run(() -> intake.setRollerVoltage(-12)))))))));
			// ------------------------ Left Trench Collect Bump Shoot
			// --------------------------------------------------
			autoChooser.addOption("Left Trench Collect Bump Shoot", Commands.runOnce(() -> {
				var startPose =
				        // drive.setPose(
				        DriverStation.getAlliance().get().equals(Alliance.Blue)
				                ? leftTrenchBumpShootAutoPath.getPathPoses().get(0)
				                : leftTrenchBumpShootAutoPath.flipPath().getPathPoses().get(0);
				if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
					startPose = new Pose2d(startPose.getTranslation(),
					        startPose.getRotation().rotateBy(Rotation2d.kPi));
				}
				drive.setPose(startPose);
			}).andThen(
			        Commands.runOnce(() -> intake.extendExtender()).alongWith(Commands.run(() -> intake.intakeFuel()))
			                .alongWith(
			                        AutoBuilder.followPath(leftTrenchBumpShootAutoPath)
			                                .andThen(primeShootCommand
			                                        .alongWith(new ShootCommand(elevator, primeShootCommand,
			                                                primeShootCommand, primeShootCommand))
			                                        .alongWith(Commands
			                                                .repeatingSequence(
			                                                        Commands.runOnce(() -> intake.setExtenderPos(
			                                                                Rotation2d.fromDegrees(30))),
			                                                        Commands.waitSeconds(1),
			                                                        Commands.runOnce(() -> intake.setExtenderPos(
			                                                                Rotation2d.fromDegrees(75))),
			                                                        Commands.waitSeconds(1))
			                                                .alongWith(Commands
			                                                        .run(() -> intake.setRollerVoltage(-12))))))));

			// -------------------------------------- Right Side Trench Collect Bump Shoot
			// --------------------------------------------------
			autoChooser.addOption("Right Trench Bump Shoot", Commands.runOnce(() -> {
				var startPose = DriverStation.getAlliance().get().equals(Alliance.Blue)
				        ? rightTrenchBumperShootAutoPath.mirrorPath().getPathPoses().get(0)
				        : rightTrenchBumperShootAutoPath.mirrorPath().flipPath().getPathPoses().get(0);
				if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
					startPose = new Pose2d(startPose.getTranslation(),
					        startPose.getRotation().rotateBy(Rotation2d.kPi));
				}
				drive.setPose(startPose);
			}).andThen(
			        Commands.runOnce(() -> intake.extendExtender()).alongWith(Commands.run(() -> intake.intakeFuel()))
			                .alongWith(
			                        AutoBuilder.followPath(rightTrenchBumperShootAutoPath.mirrorPath())
			                                .andThen(primeShootCommandEight
			                                        .alongWith(new ShootCommand(elevator, primeShootCommandEight,
			                                                primeShootCommandEight, primeShootCommandEight))
			                                        .alongWith(Commands
			                                                .repeatingSequence(
			                                                        Commands.runOnce(() -> intake.setExtenderPos(
			                                                                Rotation2d.fromDegrees(30))),
			                                                        Commands.waitSeconds(1),
			                                                        Commands.runOnce(() -> intake.setExtenderPos(
			                                                                Rotation2d.fromDegrees(75))),
			                                                        Commands.waitSeconds(1))
			                                                .alongWith(Commands
			                                                        .run(() -> intake.setRollerVoltage(-12))))))));

		} catch (FileVersionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Configure the button bindings
		configureButtonBindings();
	}
	// intake pivot 2.25 volts hard limit, both direction. //elavator 11.5 volts,
	// //intake roller 11.5
	// volts, //turret angle rotater, 1 volt max
	// shooter -11.5. volts. //hood +-1.5 volts. //Drivebase remains unchanged.
	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Default command, normal field-relative drive
		drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driverController.getLeftY(),
		        () -> -driverController.getLeftX(), () -> driverController.getRightX())); // Changed into a positive
		                                                                                  // from a negitive

		// Lock to 0° when A button is held
		driverController.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> -driverController.getLeftY(),
		        () -> -driverController.getLeftX(), () -> Rotation2d.kZero));

		// Switch to X pattern when X button is pressed
		driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

		// change rotation on motors to 0° when button on dash is pressed
		SmartDashboard.putData("Zero Robot", Commands.runOnce(() -> {
			shooter.zeroMotors();
			intake.zeroMotors();
		}).ignoringDisable(true));

		SmartDashboard.putNumber("GameTime", DriverStation.getMatchTime());

		// Reset gyro to 0° when Y button is pressed
		driverController.y().onTrue(Commands
		        .runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive)
		        .ignoringDisable(true));

		driverController.povUp().whileTrue(TuningDriveCommands.dpadDrive(drive, 0.25, 0.0));

		coDriverController.b().whileTrue(Commands.run(() -> elevator.setElevatorVoltage(-8)))
		        .onFalse(Commands.runOnce(() -> elevator.setElevatorVoltage(0)));

		// extend the extender to out position when dpad down button is pressed
		coDriverController.povDown().whileTrue(Commands.run(() -> intake.extendExtender()));
		// retract the intake when dpad up is pressed
		coDriverController.povUp().whileTrue(Commands.run(() -> intake.retractExtender()));
		// manually extends the intake to desired location via voltages
		coDriverController.rightTrigger(0.01)
		        .whileTrue(Commands.run(() -> intake.setExtenderVoltage(coDriverController.getRightTriggerAxis() * -2)))
		        .onFalse(Commands.runOnce(() -> intake.setExtenderVoltage(0)));

		coDriverController.rightBumper().whileTrue(Commands.repeatingSequence(
		        Commands.runOnce(() -> intake.setExtenderPos(Rotation2d.fromDegrees(30))), Commands.waitSeconds(1),
		        Commands.runOnce(() -> intake.setExtenderPos(Rotation2d.fromDegrees(75))), Commands.waitSeconds(1))
		        .alongWith(Commands.run(() -> intake.setRollerVoltage(-12))))
		        .onFalse(Commands.runOnce(() -> intake.setRollerVoltage(0)));

		// manually retracts the intake to desired location via voltages
		// double LTriggerVal = coDriverController.getLeftTriggerAxis();
		// intake.setExtenderVoltage(4 * LTriggerVal);
		coDriverController.leftTrigger(0.01)
		        .whileTrue(Commands.run(() -> intake.setExtenderVoltage(coDriverController.getLeftTriggerAxis() * 2)))
		        .onFalse(Commands.runOnce(() -> intake.setExtenderVoltage(0)));
		// Rolls the roller to make it intake fuel
		driverController.rightBumper().whileTrue(Commands.run(() -> intake.intakeFuel()))
		        .onFalse(Commands.runOnce(() -> intake.stopRoller()));

		// Rolls the roller to make it push out fuel
		driverController.leftBumper().whileTrue(Commands.run(() -> intake.releaseFuel()))
		        .onFalse(Commands.runOnce(() -> intake.stopRoller()));

		TuningDriveCommands.dpadDrive(drive, falseYSignal, falseXSignal);

		PrimeShootCommand primeShootCommand = new PrimeShootCommand(shooter, drive, intake, ShootingType.HUB_SHOOT);

		PrimeShootCommand simplePrimeShootCommand = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.SIMPLE_SHOOT);
		PrimeShootCommand alliancePrimeShootCommand = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.ALLIANCE_SHOOT);
		PrimeShootCommand trenchPrimeShootCommand = new PrimeShootCommand(shooter, drive, intake,
		        ShootingType.TRENCH_SHOT);

		coDriverController.povRight().whileTrue(primeShootCommand);

		coDriverController.povLeft().whileTrue(alliancePrimeShootCommand);

		coDriverController.x().whileTrue(trenchPrimeShootCommand);

		coDriverController.leftBumper().whileTrue(simplePrimeShootCommand);

		coDriverController.a().whileTrue(
		        new ShootCommand(elevator, primeShootCommand, simplePrimeShootCommand, alliancePrimeShootCommand));
	}

	public static double getTimeLeftUntilSwitch() {
		var currentTime = DriverStation.getMatchTime();
		if (DriverStation.isAutonomous()) {
			return currentTime;
		} else if (currentTime > 130) {
			return currentTime - 130;
		} else if (currentTime > 105) {
			return currentTime - 105;
		} else if (currentTime > 80) {
			return currentTime - 80;
		} else if (currentTime > 55) {
			return currentTime - 55;
		} else if (currentTime > 30) {
			return currentTime - 30;
		} else {
			return Math.max(0, currentTime);
		}
	}

	public static boolean isBlueShift() {
		String allianceLead = DriverStation.getGameSpecificMessage();
		var currentTime = DriverStation.getMatchTime();
		boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
		boolean output;
		if (DriverStation.isAutonomous()) {
			output = isBlue;
		} else if (currentTime > 130) {
			output = isBlue;
		} else if (currentTime > 105) {
			output = allianceLead.equals("B");
		} else if (currentTime > 80) {
			output = !allianceLead.equals("B");
		} else if (currentTime > 55) {
			output = allianceLead.equals("B");
		} else if (currentTime > 30) {
			output = !allianceLead.equals("B");
		} else {
			output = isBlue;
		}
		return output;
	}

	public static boolean isOurShift() {
		String allianceLead = DriverStation.getGameSpecificMessage();
		var currentTime = DriverStation.getMatchTime();
		boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
		boolean output;
		if (DriverStation.isAutonomous()) {
			output = true;
		} else if (currentTime > 130) {
			output = true;
		} else if (currentTime > 105) {
			output = allianceLead.equals("B") == isBlue;
		} else if (currentTime > 80) {
			output = !allianceLead.equals("B") == isBlue;
		} else if (currentTime > 55) {
			output = allianceLead.equals("B") == isBlue;
		} else if (currentTime > 30) {
			output = !allianceLead.equals("B") == isBlue;

		} else {
			output = true;
		}
		return output;
	}

	public Pose2d getDrivePose() {
		return drive.getPose();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

}
