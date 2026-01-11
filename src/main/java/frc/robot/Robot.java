// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*
 * CAN IDs
 * 
 * 0 - PDP
 * 1 - FL DT motor (Victor SPX)
 * 2 - BL DT motor (Victor SPX)
 * 3 - FR DT motor (Victor SPX)
 * 4 - BR DT motor (Victor SPX)
 */

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.WPI_CallbackHelper;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final WPI_VictorSPX m_leftFrontDrive = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_leftBackDrive= new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_rightFrontDrive = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_rightBackDrive= new WPI_VictorSPX(4);


  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftFrontDrive,m_rightFrontDrive);

  private final PS4Controller m_controller = new PS4Controller(0);
  private final Timer m_timer = new Timer();

  
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    //   // Drive forwards half speed, make sure to turn input squaring off
    //   m_robotDrive.arcadeDrive(0.5, 0.0, false);
    // } else {
    //   m_robotDrive.stopMotor(); // stop robot
    // }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() 
  {

    m_leftFrontDrive.follow(m_leftBackDrive);
    m_rightFrontDrive.follow(m_rightBackDrive);
    m_rightFrontDrive.setInverted(true);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

    // m_leftFrontDrive.follow(m_leftBackDrive);
    // m_rightFrontDrive.follow(m_rightBackDrive);

    // // Just for testing
    // m_rightBackDrive.follow(m_leftBackDrive);

    // m_timer.restart();
    // m_leftBackDrive.set(VictorSPXControlMode.PercentOutput, 1);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // if (m_timer.get() > 1) {
    //   m_leftBackDrive.set(VictorSPXControlMode.PercentOutput, 0);
    // }
  }
}