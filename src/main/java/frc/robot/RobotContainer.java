package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Eject;
import frc.robot.commands.Extendo;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SpinUp;
import frc.robot.commands.Stop;
import frc.robot.subsystems.swervedrive.AlignerSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
//   private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(0); // Port 0
  private final CommandXboxController operatorXbox =
      new CommandXboxController(1); // Port 1
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final AlignerSubsystem aligner = new AlignerSubsystem();
  private final CANFuelSubsystem shooter = new CANFuelSubsystem(aligner);
  private final IntakeSubsystem intake = new IntakeSubsystem();


//     // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(0.05)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  // private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // register named commands
    NamedCommands.registerCommand("spin up and launch", new LaunchSequence(shooter));
    NamedCommands.registerCommand("get out of my way", new Extendo(intake));


    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();
    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("bule");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.rightTrigger().whileTrue(drivebase.driveRobotOriented(driveAngularVelocity));
    // MAGIC AUTO-ALIGN
    driverXbox.leftTrigger().whileTrue(drivebase.driveRobotOriented(driverXbox.getLeftY() * -1,
                                                                driverXbox.getLeftX() * -1,
                                                                aligner.getTurnAmount()));

    driverXbox.y().whileTrue(drivebase.aimAtTarget());
    //driverXbox.x().whileTrue(drivebase.properDistanceFromTarget());
    driverXbox.start().whileTrue(drivebase.resetGyro());
    intake.setDefaultCommand(intake.StopEveryMotor());
    
    // Shooter controls
    operatorXbox.leftTrigger().onTrue(
      shooter.toggleState());
    operatorXbox.rightTrigger().whileTrue(
      shooter.setState(CANFuelSubsystem.ShooterState.SHOOT));
    operatorXbox.rightTrigger().onFalse(shooter.setState(CANFuelSubsystem.ShooterState.SPIN_UP));

    // operatorXbox.povDown().onTrue(intake.IntakeReversed());
    // operatorXbox.povRight().onTrue(intake.StopJustRoller());
    operatorXbox.a().whileTrue(intake.MoveIn());
    operatorXbox.b().whileTrue(intake.MoveOut());
    operatorXbox.x().whileTrue(intake.Feedout());
    operatorXbox.y().whileTrue(intake.FEEEED());

    operatorXbox.start().whileTrue(Commands.parallel(new Stop(shooter), intake.StopEveryMotor()));

    // Intake comman
    // driverXbox.povUp(RightThumbstick).onTrue(shooter.AimUp(RightBumper));
    // driverXbox.povDown(RightThumbstick).onTrue(shooter.AimDown(LeftBumper));

    // Schedule `ExampleCommand` when `exampleCondition
  //  changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is being pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  

}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(drivebase);
    // return Autos.pathPlannedAuto();
    return autoChooser.getSelected();
    // return Commands.none();
  }
}
