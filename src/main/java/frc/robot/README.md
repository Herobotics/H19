# FRC Team 2500 Code

Contains important notes for contributing to this codebase.

pre-Monday code to add:
- [ ] move the printlns to use the dashboard so Glass can use it
- [ ] set direction based on autonomous chosen
- [ ] add sample distance vs rps table
- [ ] add area vs distance code
- [ ] add functionality to "hold" intake in when buttons aren't held
- [ ] add direction following calculations following https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html
- [ ] simulate autonomous in glass to diagnose

## Monday's punch list
--- enabled tasks
- [ ] Tune position PID for intake
- [ ] Add "keep same position" on let go of button to prevent gravity + test
- [ ] Determine distance vs shooting speed graph
--- disabled tasks
- [ ] Update Limelight position relative to robot
- [ ] Make pipelines for AprilTag ID for just the Hub
- [ ] Make distance vs apriltag area graph
--- enabled tasks
- [ ] tune direction following apriltag kP
- [ ] test it!


Necessary TODOs:
- [ ] Check that navgrid.json is correct
- [ ] Update Limelight position relative to robot
- [ ] Test limelight localization on both sides of the field
- [ ] Test autos on both sides of the field

Recommended TODOs:
- [ ] Add telemetry data about the AprilTags to a dashboard
- [ ] Verify the robot can accurately drive 3m straight and turn 360 degrees. If broken, tune PID.
- [ ] Make collect and shoot Autos
- [ ] Add set heading to the autonomous init
- [ ] Figure out the rpm-to-distance curve
- [ ] Add distance detection to apriltags
- [ ] Make a pipeline for blue shooting; make a pipeline for red shooting
- [ ] Figure out shoot-on-the-move parameters: what angle for a given speed compensates?
- [ ] Add functionality to strafe while shooting? Put it on the dpad?

In case of broken, TODOs:
- [ ] Figure out if late canbus frames are a problem
- [ ] Calibrate the gyroscope. We did this last year, was it in Phoenix Tuner X?
- [ ] Calibrate/characterize the robot's PID loop with SysId
- [ ] Limit which AprilTags we are looking at to just the centered ones.
- [ ] Check that our odometry has an always-blue-corner origin, which is compatible with PathPlanner
- [ ] Add the rest of the [Limelight MegaTag calibration steps](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#6-special-apriltag-functionality)

## More about Command Based Programming

- [WPILib documentation](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [WPIlib Template](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/templates/commandbased)

## More about Swerve Drive

This year we are using [YAGSL](https://docs.yagsl.com/) "Yet Another Generic Swerve Library". Mostly following [Drivebase 2026 Example](https://github.com/Yet-Another-Software-Suite/YAGSL/blob/main/examples/drivebase_only_2026/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java) but there's a lot of extra code within that so we pared it down.

### Our physical setup
We use 8 Kraken x60s and Mk4i SDS modules (from fall 2024) with CANCoders and a Pigeon 2.0 gyro and Colson wheels.

### Odometry
Keeping track of where we are can be done with the gyroscope. Adding in pose estimation from cameras can be done by using AddVisionMeasurement (https://docs.yagsl.com/overview/our-features/vision-odometry). It is YAGSL's version of WPILib's [SwerveDrivePoseEstimator](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html).

Vision tracking with the Limelight camera can be added by following the [Limelight Documentation](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib). We probably want to use [MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation) to estimate pose. And [here's a discussion](https://www.chiefdelphi.com/t/how-to-use-sample-limelight-code-in-java-command-based-file/372663/6) on how to make that periodic function work in a command based system.

### PathPlanner
-[Documentation](https://pathplanner.dev/home.html)