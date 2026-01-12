package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeCommands.ROLLER_VOLTS;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class ShooterCommands {


    /** Intake shooter preset voltages. */
  public final class SHOOTER_VOLTS {
    public static final LoggedTunableNumber SHOOTAUTO =
        new LoggedTunableNumber("ShootAutoSpeed", -6);
    public static final LoggedTunableNumber FARSHOOT =
        new LoggedTunableNumber("ShootFarSpeed", -1.5);
    public static final LoggedTunableNumber EMERGENCY =
        new LoggedTunableNumber("Emergency Outtake", -12);
    public static final LoggedTunableNumber STOP = new LoggedTunableNumber("Stop ", 0);
  }

  public final class FEEDER_VOLTS {
    public static final LoggedTunableNumber AUTO =
        new LoggedTunableNumber("AutoSpeed", -6);
    public static final LoggedTunableNumber STOW =
        new LoggedTunableNumber("OuttakeSpeed", -1.5);
    public static final LoggedTunableNumber EMERGENCY =
        new LoggedTunableNumber("Emergency Outtake", -12);
    public static final LoggedTunableNumber STOP = new LoggedTunableNumber("Stop ", 0);
  }

  /**
   * Deploys the intake by moving the rotation motor to the down position and setting the roller
   * motor to intake speed.
   */
  public static Command shootAuto(Flywheel rollerMotor) {
    return Commands.parallel(
        new FlywheelVoltageCommand(rollerMotor, SHOOTER_VOLTS.SHOOTAUTO));
  }


}
