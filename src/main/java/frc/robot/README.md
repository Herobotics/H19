# FRC Team 2500 Code

Contains important notes for contributing to this codebase.

## Important code change history

1. Generated sample TimedRobot
2. Added YAGSL driving and configs
3. Converted to Command Based Programming
4. Added Autonomous code
5. Added Shooter subsystem

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