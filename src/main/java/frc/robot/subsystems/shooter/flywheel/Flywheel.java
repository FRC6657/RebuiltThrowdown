// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /**
   * @param io the hardware IO implementation (real or simulated)
   */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void changeSetpoint(double setpoint) {
    io.changeSetpoint(setpoint);
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
