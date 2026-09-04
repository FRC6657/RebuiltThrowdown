package frc.robot.subsystems.indexer.floor;

import org.littletonrobotics.junction.AutoLog;

public interface FloorIO {

  @AutoLog
  public static class FloorIOInputs {
    public double motorOneTemp = 0.0; // Celsius
    public double motorOneVoltage = 0.0; // Volts
    public double motorOneStatorCurrent = 0.0; // Amps

    public double motorTwoTemp = 0.0; // Celsius
    public double motorTwoVoltage = 0.0; // Volts
    public double motorTwoStatorCurrent = 0.0; // Amps
  }

  public default void updateInputs(FloorIOInputs inputs) {}

  public default void changeSetpoint(double newSetpoint) {}
}
