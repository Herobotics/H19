package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
 /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  
  private final double maximumSpeed = Units.feetToMeters(3.0);

public SwerveSubsystem(){
  try
    {
            Pose2d startingPose = new Pose2d(new Translation2d(Meter.of(2),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0));
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      File directory = new File(Filesystem.getDeployDirectory(),"swerve");
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, startingPose);
      System.out.println(swerveDrive.swerveDriveConfiguration.toString());
      swerveDrive.setHeadingCorrection(false);
      swerveDrive.setCosineCompensator(false);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    
}

public Command drive(
      double xSpeed, double ySpeed, double rot) {
        return run(() -> {
        ChassisSpeeds cs = new ChassisSpeeds(xSpeed, ySpeed, rot);
    swerveDrive.driveFieldOriented(cs);
    });
      }

public Command driveFieldOriented(SwerveInputStream driveAngularVelocity) {
  return run(() -> {
    swerveDrive.driveFieldOriented(driveAngularVelocity.get());
  });
}
  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

}
