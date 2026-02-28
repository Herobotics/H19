package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
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
  private final boolean     useVisionCalibration = true;
  
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

    setupPathPlanner();

}

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }


    /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

public Command drive(
      double xSpeed, double ySpeed, double rot) {
        return run(() -> {
        ChassisSpeeds cs = new ChassisSpeeds(xSpeed, ySpeed, rot);
    swerveDrive.driveFieldOriented(cs);
    });
      }

public Command driveForward(Distance distance) {
  return Commands.none();

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

  // Necessary for PathPlanner
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
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
      LimelightHelpers.SetRobotOrientation(Constants.limelight_name, swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.limelight_name);
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
        System.err.println("Pose estimate X: " + mt2.pose.getMeasureX() + " Y:" + mt2.pose.getMeasureY());
        // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
        //     mt2.pose,
        //     mt2.timestampSeconds);
      }
    }
  }


  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  private double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX(Constants.limelight_name) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= maximumSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  private double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY(Constants.limelight_name) * kP;
    targetingForwardSpeed *= maximumSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  /** Aim the robot at the target AprilTag.
   *
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget()
  {
    return run(() -> {
      // Only run this if we can see a target
      if (LimelightHelpers.getTV(Constants.limelight_name)) {
        // add check of whether there are any targets
        final double rot_limelight = limelight_aim_proportional();
        final double forward_limelight = limelight_range_proportional();
        final Translation2d movement = new Translation2d(forward_limelight, 0);
        // consider driving in open loop for this
        swerveDrive.drive(movement, rot_limelight, false, false);
      } else {
        System.out.println("Cannot see a target");
      }


    });
  }

    /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }

    /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

    /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

}
