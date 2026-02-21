# FRC Team 2500 Code

Contains important notes for contributing to this codebase.

## Important code change history

1. Generated sample TimedRobot
2. Added YAGSL driving and configs
3. Converted to Command Based Programming
4. Added Autonomous code
5. Added Shooter subsystem

Necessary TODOs:
[] Design intake and shooter controls (Devonair)
[] Add EveryBot code for controlling the fuel ([subsystem code](https://github.com/Robonauts-Everybot/FRC-Everybot-2026-Code/blob/main/src/main/java/frc/robot/subsystems/CANFuelSubsystem.java))/[command code](https://github.com/Robonauts-Everybot/FRC-Everybot-2026-Code/tree/main/src/main/java/frc/robot/commands) & modify it for our usages
[] Add AprilTag tracking to our swerve drive
[] Hook up Limelight camera and give it the right software
[] Upload a field map to our Limelight

Recommended TODOs:
[] Verify the robot can accurately drive 3m straight and turn 360 degrees. If broken, tune PID.
[] Make some sort of PathPlanner Autonomous path
[] Add an "auto-align target" button

In case of broken, TODOs:
[] Calibrate the gyroscope. We did this last year, was it in Phoenix Tuner X?
[] Add a "reset gyroscope" button
[] Update Limelight position relative to robot
[] Calibrate/characterize the robot's PID loop with SysId
[] Limit which AprilTags we are looking at to just the centered ones.

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