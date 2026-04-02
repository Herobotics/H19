// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity. 
 */
public final class Constants {
  public static final String limelight_name = "limelight-benji";

  public static final class AprilTag {
    public static final double AREA_FACTOR = 52.0;
    public static final double MIN_DISTANCE = 60.0;
    public static final double MAX_DISTANCE = 150.0;
    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 10;
    public static final double IDEAL_ANGLE = 4.0;

    public static final double K_ALIGN_P = 0.01;
  }

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 47;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 60;
    public static final int INDEXER_MOTOR_ID = 59;

    // Current limit for fuel mechanism motors.
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;

    // All values likely need to be tuned based on your robot
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;
    public static final double LAUNCHING_LAUNCHER_RPS = 75;

    public static final double SPIN_UP_SECONDS = 0.75;
    // TODO MAKE SURE THEY ALL GET OUT
    public static final double SHOOT_AUTO_SECONDS = 4.0;

    public static final double LAUNCHER_KV = 0.118;
    public static final double LAUNCHER_KP = 0.5;
  }

  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }

  public static final class IntakeConstants {
    // Motor controller IDs for Intake motors
    public static final int INTAKE_MOVER_MOTOR_ID = 55;
    public static final int INTAKE_ROLLER_MOTOR_ID = 56;

    public static final double ROLLER_MOTOR_INTAKE_PERCENT = -1.0;
    public static final double MOVER_MOTOR_EXTENSION_VOLTAGE = -1.0;
    public static final double MOVER_MOTOR_RETRACT_VOLTAGE = 1.5;
    // TODO CHECK THIS
    public static final double MOVER_MOTOR_EXTENSION_POSITION = -5.5;
    public static final double MOVER_MOTOR_RETRACT_POSITION = 0;
  }
}