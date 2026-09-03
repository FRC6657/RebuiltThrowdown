package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;

import java.io.IOError;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase{
    
    public IntakeIO io;
    public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
        @param io

    public Intake(IntakeIO io) {
        this.io = io;
    }

        @param setpoint
        @return

    public Command changeSetpoint(ExtensionSetpoint setpoint) {
        return this.runOnce(() -> io.changeSetpoint(setpoint));
    }

        @param setpoint
        @return

    public Command changeSetpoint(double setpoint) {
        return this.runOnce(() -> io.changeSetpoint(setpoint));
    }

        @return
    
    @AutoLogOutput(key = "AtSetpoint/Intake")
    public boolean atSetpoint() {
        return io.atSetpoint();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
