// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {
  //not using just yet lol


  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25));
  }
}
