// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  // private PositionJoint intakeRotationMotor =
  // new PositionJoint(
  //   new PositionJointIOSparkMax("intakeRotationMotor",
  //    PositionJointConstants.INTAKE_PIVOT),
  //    PositionJointConstants.INTAKE
  // );
  private Flywheel intakeRollerMotor =
  new Flywheel(
    new FlywheelIOSparkMax("intakeRollerMotor",
    FlywheelConstants.INTAKE_FLYWHEEL),
    FlywheelConstants.INTAKE_ROLLER);

  private Flywheel shooterMotor = new Flywheel(new FlywheelIOSparkMax("shooterMotor", FlywheelConstants.SHOOTER_FLYWHEEL), FlywheelConstants.SHOOTER);
  private Flywheel indexMotor = new Flywheel(new FlywheelIOSparkMax("indexMotor", FlywheelConstants.FEEDER_FLYWHEEL), FlywheelConstants.INTAKE_ROLLER);
  private PositionJoint intakePivotMotor = new PositionJoint(new PositionJointIOSparkMax("positionPivotMotor", PositionJointConstants.INTAKE_PIVOT), PositionJointConstants.INTAKE);
  private PositionJoint hoodMotor = new PositionJoint(new PositionJointIOSparkMax("hoodMotor", PositionJointConstants.SHOOTER_HOOD), PositionJointConstants.HOOD);
  


  // The driver's controller
  private final  CommandPS4Controller driverController = new CommandPS4Controller(
      DRIVER_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> driverController.getLeftY() * DRIVE_SCALING,
            () -> -driverController.getRightX() * ROTATION_SCALING));

    shooterMotor.setDefaultCommand(new FlywheelVoltageCommand(shooterMotor, () -> (driverController.getL2Axis() + 1) * 6));
    hoodMotor.setDefaultCommand(new RunCommand(() -> hoodMotor.setVoltage(driverController.getRightY() * 12), hoodMotor));

    //  // Coral Intake
    driverController.R1().whileTrue(new FlywheelVoltageCommand(intakeRollerMotor, ()->8)).whileFalse(new FlywheelVoltageCommand(intakeRollerMotor, ()->0));
    
    driverController.cross().whileTrue(new RunCommand(() -> intakePivotMotor.setVoltage(4), intakePivotMotor)).whileFalse(new RunCommand(() -> intakePivotMotor.setVoltage(-4), intakePivotMotor));
    driverController.R2().whileTrue(new FlywheelVoltageCommand(indexMotor, ()->-12)).whileFalse(new FlywheelVoltageCommand(indexMotor, ()->0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
