// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX LeftIntakeLauncher;
  private final TalonFX RightIntakeLauncher;
  private final SparkMax Indexer;

  public enum ShooterState {
    SPIN_UP,
    STAHP,
    SHOOT
  }

  private ShooterState state;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // // create motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID);
    RightIntakeLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID);
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);
    state = ShooterState.STAHP;

    // // create the configuration for the feeder roller, set a current limit and apply
    // // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  public void setMotorState() {
    if (this.state == ShooterState.SHOOT) {
      System.out.println("Shooter state: SHOOT");
      this.setIntakeLauncherRoller(
            SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
      this.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
    } else if (this.state == ShooterState.SPIN_UP) {
      System.out.println("Shooter state: SPIN_UP");
          this
        .setIntakeLauncherRoller(
            SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
      this.setFeederRoller(SmartDashboard.getNumber("Launching spin-up feeder value", INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT));
    } else if (this.state == ShooterState.STAHP) {
      System.out.println("Shooter state: STAHP");
      this.setIntakeLauncherRoller(0);
      this.setFeederRoller(0);
    }
  }

  public Command setState(ShooterState new_state) {
    return Commands.runOnce(() -> {
      this.state = new_state;
    });
  }

  public Command toggleState() {
    return Commands.runOnce(() -> {
      System.out.println("Running... once?");
      if (this.state == ShooterState.STAHP) {
        this.state = ShooterState.SPIN_UP;
      } else { // spin up or shooting
        this.state = ShooterState.STAHP;
      }
    });
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    LeftIntakeLauncher.set(power);
    RightIntakeLauncher.set(-1.0 * power); // negative for clockwise
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    Indexer.set(0);
    LeftIntakeLauncher.set(0);
    RightIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    this.setMotorState();
    // This method will be called once per scheduler run
  }
} 