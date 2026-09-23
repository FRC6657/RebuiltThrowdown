package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class FlywheelIO_Sim implements FlywheelIO {
  private TalonFX flywheelMotor = new TalonFX(GlobalConstants.CAN.Shooter_Flywheel.id);

  private VelocityVoltage setpoint = new VelocityVoltage(0);

  private DCMotorSim flywheelModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, FlywheelConstants.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  public FlywheelIO_Sim() {
    flywheelMotor
        .getConfigurator()
        .apply(
            FlywheelConstants.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)));

    var velocity = flywheelMotor.getVelocity();
    var acceleration = flywheelMotor.getAcceleration();
    var temp = flywheelMotor.getDeviceTemp();
    var voltage = flywheelMotor.getMotorVoltage();
    var statorCurrent = flywheelMotor.getSupplyCurrent();

    velocity.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    acceleration.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    temp.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY / 4d);
    voltage.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    statorCurrent.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);

    flywheelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    if (setpoint.Velocity == 0) {
      flywheelMotor.setControl(new VoltageOut(0));
    } else {
      flywheelMotor.setControl(setpoint);
    }

    var flywheelSim = flywheelMotor.getSimState();
    flywheelSim.setSupplyVoltage(12);
    flywheelModel.setInputVoltage(flywheelSim.getMotorVoltage());
    flywheelModel.update(1 / GlobalConstants.MAIN_LOOP_FREQUENCY);
    var angularPosition = flywheelModel.getAngularPosition().times(FlywheelConstants.GEAR_RATIO);
    var angularVelocity = flywheelModel.getAngularVelocity().times(FlywheelConstants.GEAR_RATIO);
    flywheelSim.setRawRotorPosition(angularPosition);
    flywheelSim.setRotorVelocity(angularVelocity);

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
