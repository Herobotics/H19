package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
// 

  public static Command exampleAuto(SwerveSubsystem drivebase) {
    return Commands.sequence(
        drivebase.drive(1.0, 0, 0).withTimeout(5.0),
        drivebase.drive(0, 1.0, 0)
        );
  }

  public static Command pathPlannedAuto() {
    return new PathPlannerAuto("New Auto");
  }

  private Autos() {  // Don't make an instance of this
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
