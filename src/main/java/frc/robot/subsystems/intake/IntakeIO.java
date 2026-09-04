package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {

        public double extensionPosition = 0.0; //Inches
        public double extensionVelocity = 0.0; //Inches per second
        public double extensionAcceleration = 0.0; //Inches per second per second
        public double extensionTemp; //Celsius
        public double extensionVoltage = 0.0; //Volts
        public double extensionStatorCurrent = 0.0;  //Amps

        public double rollerTemp = 0.0; //Celsius
        public double rollerVoltage = 0.0; //Volts
        public double rollerStatorCurrent = 0.0; //Amps
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void changeSetpoint(ExtensionSetpoint setpoint) {}

    public default void changeSetpoint(double setpoint) {}
    
    public default boolean atSetpoint() {
        return false;
    }
}
