package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public IntakeIO io;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command changeSetpointP(double pivotSetpoint) {
    return this.runOnce(() -> io.changeSetpointP(pivotSetpoint));
  }

  public Command changeSetpointR(double rollerSetpoint) {
    return this.runOnce(() -> io.changeSetpointR(rollerSetpoint));
  }

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
