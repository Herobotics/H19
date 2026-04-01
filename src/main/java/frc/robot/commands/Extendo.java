// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Extendo extends Command {
  /** Creates a new Intake. */

  IntakeSubsystem intakeSubsystem;

  public Extendo(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    System.out.println("Starting move out");
    intakeSubsystem.MoveOutAuto();
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.FEEEDAuto();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double setpoint = Constants.IntakeConstants.MOVER_MOTOR_EXTENSION_POSITION;
    return (intakeSubsystem.HowFarOutIsTheIntake() < (setpoint + 0.2)) &&
           (intakeSubsystem.HowFarOutIsTheIntake() > (setpoint - 0.1));
  }
}