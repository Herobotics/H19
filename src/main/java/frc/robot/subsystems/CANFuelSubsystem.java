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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.AlignerSubsystem;

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

  private final AlignerSubsystem aligner;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem(AlignerSubsystem aligner) {
    // // create motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID);
    RightIntakeLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID);
    LeftIntakeLauncher.getConfigurator().apply(new Slot0Configs().withKV(Constants.FuelConstants.LAUNCHER_KV).withKP(Constants.FuelConstants.LAUNCHER_KP));
    RightIntakeLauncher.getConfigurator().apply(new Slot0Configs().withKV(Constants.FuelConstants.LAUNCHER_KV).withKP(Constants.FuelConstants.LAUNCHER_KP));
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);
    state = ShooterState.STAHP;

    this.aligner = aligner;

    // // create the configuration for the feeder roller, set a current limit and apply
    // // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_RPS);
    SmartDashboard.putString("Shooter state", "INIT");
    SmartDashboard.putNumber("Shooter RPS", -1);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  public void setMotorState() {
    double desired_rps = 
            SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_RPS);
    if (this.aligner.isAligned()) {
      desired_rps = this.aligner.getRPS();
    }
    if (this.state == ShooterState.SHOOT) {
      SmartDashboard.putString("Shooter state", "SHOOT");
      this.setIntakeLauncherRoller(desired_rps);
      this.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
    } else if (this.state == ShooterState.SPIN_UP) {
      SmartDashboard.putString("Shooter state", "SPIN_UP");
          this
        .setIntakeLauncherRoller(desired_rps);
      this.setFeederRoller(SmartDashboard.getNumber("Launching spin-up feeder value", INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT));
    } else if (this.state == ShooterState.STAHP) {
      SmartDashboard.putString("Shooter state", "STAHP");
      this.stop();
    }
    SmartDashboard.putNumber("Shooter RPS", this.RightIntakeLauncher.getVelocity().getValueAsDouble());
  }

  // Needed to bind to buttons
  public Command setState(ShooterState new_state) {
    return Commands.runOnce(() -> {
      this.state = new_state;
    });
  }

  // Non-Command version
  public void setStateRegular(ShooterState new_state) {
    this.state = new_state;
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
  public void setIntakeLauncherRoller(double rps) {
    LeftIntakeLauncher.setControl(new VelocityVoltage(rps));
    RightIntakeLauncher.setControl(new VelocityVoltage(-1.0 * rps));
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    this.setStateRegular(ShooterState.STAHP);
    Indexer.set(0);
    LeftIntakeLauncher.setVoltage(0);
    RightIntakeLauncher.setVoltage(0);
  }

  @Override
  public void periodic() {
    this.setMotorState();
    // This method will be called once per scheduler run
  }
} 