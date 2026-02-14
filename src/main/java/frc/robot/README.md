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