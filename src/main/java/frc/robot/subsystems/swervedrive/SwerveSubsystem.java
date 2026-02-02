package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem {
 /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  

  private final double maximumSpeed = Units.feetToMeters(3.0);

public SwerveSubsystem(){
  try
    {
      File directory = new File(Filesystem.getDeployDirectory(),"swerve");
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
}

public void drive(
      double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds cs = new ChassisSpeeds(ySpeed, ySpeed, rot);
    swerveDrive.drive(cs);
      }
}
