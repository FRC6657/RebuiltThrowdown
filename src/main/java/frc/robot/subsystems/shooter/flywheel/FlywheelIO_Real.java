package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIO_Real implements FlywheelIO {
  private TalonFX flywheelMotor = new TalonFX(GlobalConstants.CAN.Shooter_Flywheel.id);

  private VelocityVoltage setpoint = new VelocityVoltage(0);

  public FlywheelIO_Real() {

    flywheelMotor.getConfigurator().apply(FlywheelConstants.CONFIG);

    var velocity = flywheelMotor.getVelocity();
    var acceleration = flywheelMotor.getAcceleration();
    var temp = flywheelMotor.getDeviceTemp();
    var voltage = flywheelMotor.getMotorVoltage();
    var statorCurrent = flywheelMotor.getSupplyCurrent();

    velocity.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    acceleration.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    flywheelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    if (setpoint.Velocity == 0) {
      flywheelMotor.setControl(new VoltageOut(0));
    } else {
      flywheelMotor.setControl(setpoint);
    }

    inputs.velocity = flywheelMotor.getVelocity().getValueAsDouble() * 60d;
    inputs.acceleration = flywheelMotor.getAcceleration().getValueAsDouble() * 60d;

    inputs.temp = flywheelMotor.getDeviceTemp().getValueAsDouble();
    inputs.voltage = flywheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = flywheelMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint.Velocity = (setpoint / 60d); // Convert RPM to rotations per second
  }
}
