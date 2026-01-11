// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {

  private final WPI_VictorSPX m_leftFrontDrive;
  private final WPI_VictorSPX m_leftBackDrive;
  private final WPI_VictorSPX m_rightFrontDrive;
  private final WPI_VictorSPX m_rightBackDrive;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {

    m_leftFrontDrive = new WPI_VictorSPX(LEFT_LEADER_ID);
    m_leftBackDrive = new WPI_VictorSPX(LEFT_FOLLOWER_ID);
    m_rightFrontDrive = new WPI_VictorSPX(RIGHT_LEADER_ID);
    m_rightBackDrive = new WPI_VictorSPX(RIGHT_FOLLOWER_ID);

    m_leftBackDrive.follow(m_leftFrontDrive);
    m_rightBackDrive.follow(m_rightFrontDrive);

    m_rightFrontDrive.setInverted(true);

    drive = new DifferentialDrive(m_leftFrontDrive, m_rightFrontDrive);
  }

  @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
