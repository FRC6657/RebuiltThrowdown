package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  /** Logged sensor inputs for the flywheel leader and follower motors. */
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocity = 0.0; // RPM
    public double acceleration = 0.0; // RPM per second
    public double temp = 0.0; // Celsius
    public double voltage = 0.0; // Volts
    public double statorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies motor outputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets the flywheel target velocity in RPM. */
  public default void changeSetpoint(double setpoint) {}
}
