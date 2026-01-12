package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 2.0, 40, "");

  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  public static final FlywheelHardwareConfig INTAKE_FLYWHEEL =
      new FlywheelHardwareConfig(new int[] {8}, new boolean[] {true}, 16/40, 30, "");

  public static final FlywheelHardwareConfig SHOOTER_FLYWHEEL =
      new FlywheelHardwareConfig(new int[] {9}, new boolean[] {true}, 1, 40, "");

  public static final FlywheelHardwareConfig FEEDER_FLYWHEEL =
      new FlywheelHardwareConfig(new int[] {11}, new boolean[] {true}, 16/40, 30, "");


  public static final FlywheelGains INTAKE_ROLLER =
      new FlywheelGains(0, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  public static final FlywheelGains SHOOTER =
      new FlywheelGains(0, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  public static final FlywheelGains FEEDER =
      new FlywheelGains(0, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);
}
