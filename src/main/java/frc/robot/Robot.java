// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final SwerveSubsystem swerve_system = new SwerveSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  private Field2d m_field = new Field2d();

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SmartDashboard.putData("Field", m_field);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 10.0) {
      swerve_system.drive(0.0, 0.0, -1.0);
    } else {
      swerve_system.drive(0, 0, 0);
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    //System.out.println("Hello world");
    swerve_system.drive(-m_controller.getLeftY(), -m_controller.getLeftX(), m_controller.getRightX());
    // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRight());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
