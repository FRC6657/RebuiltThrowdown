package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {

    public double pivotPosition = 0.0; // Inches
    public double pivotVelocity = 0.0; // Inches per second
    public double pivotAcceleration = 0.0; // Inches per second per second
    public double pivotTemp; // Celsius
    public double pivotVoltage = 0.0; // Volts
    public double pivotStatorCurrent = 0.0; // Amps

    public double rollerTemp = 0.0; // Celsius
    public double rollerVoltage = 0.0; // Volts
    public double rollerStatorCurrent = 0.0; // Amps
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changeSetpointP(double setpoint) {}

  public default void changeSetpointR(double setpoint) {}

  public default boolean atSetpoint() {
    return false;
  }
}
