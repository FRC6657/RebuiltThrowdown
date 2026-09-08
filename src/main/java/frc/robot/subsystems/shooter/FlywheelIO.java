package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double motorOnePosition = 0.0; // Rotations
    public double motorOneVelocity = 0.0; // Rotations per second
    public double motorOneTemp = 0.0; // Celsius
    public double motorOneVoltage = 0.0; // Volts
    public double motorOneCurrent = 0.0; // Amps

    public double motorTwoPosition = 0.0; // Rotations
    public double motorTwoVelocity = 0.0; // Rotations per second
    public double motorTwoTemp = 0.0; // Celsius
    public double motorTwoVoltage = 0.0; // Volts
    public double motorTwoCurrent = 0.0; // Amps
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
