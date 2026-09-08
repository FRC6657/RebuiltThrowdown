// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Floor extends SubsystemBase {

  private final FloorIO io;

  private final FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

  public Floor(FloorIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
  }
}
