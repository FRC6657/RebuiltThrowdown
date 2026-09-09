package frc.robot.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  /** Logged sensor inputs for the hood motor. */
  @AutoLog
  public static class PivotIOInputs {
    public double position = 0.0; // Degrees
    public double temp = 0.0; // Celsius
    public double voltage = 0.0; // Volts
    public double statorCurrent = 0.0; // Amps
  }

  /** Reads the latest sensor values and applies the position control output. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the pivot to the target angle in degrees (clamped to min/max). */
  public default void changeSetpoint(double setpoint) {}
}
