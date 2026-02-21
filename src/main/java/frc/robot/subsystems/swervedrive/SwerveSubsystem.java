package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
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

    /**
   * Enable vision odometry updates while driving.
   */
  private final boolean     useVisionCalibration = false;
 
  
  private final double maximumSpeed = Units.feetToMeters(0.5);

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
      // swerveDrive.setHeadingCorrection(false);
      // swerveDrive.setCosineCompensator(false);

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

    @Override
  public void periodic()
  {
    swerveDrive.updateOdometry();
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (useVisionCalibration)
    {
      updatePositionWithLimelight();
    }
  }

  public void updatePositionWithLimelight() {
    boolean doRejectUpdate = false;
    
    {
      LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      // We don't drive our robot like this
      // if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      // {
      //   doRejectUpdate = true;
      // }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

}
