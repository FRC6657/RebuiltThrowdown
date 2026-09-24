package frc.robot.subsystems.shooter.pivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.PivotConstants;

public class PivotIO_Sim implements PivotIO {
    private TalonFX pivotMotor = new TalonFX(GlobalConstants.CAN.Shooter_Pivot.id);

  private PositionVoltage positionVoltage =
      new PositionVoltage(PivotConstants.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);

    private DCMotorSim pivotModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, PivotConstants.GEAR_RATIO), DCMotor.getFalcon500(1));

  public PivotIO_Sim() {
    pivotMotor
        .getConfigurator()
        .apply(
            ShooterConstants.PivotConstants.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)));
    pivotMotor.setPosition(ShooterConstants.PivotConstants.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    pivotMotor.setControl(positionVoltage);

    var pivotSim = pivotMotor.getSimState();
    pivotSim.setSupplyVoltage(12);
    pivotModel.setInputVoltage(pivotSim.getMotorVoltage());
    pivotModel.update(1 / GlobalConstants.MAIN_LOOP_FREQUENCY);
    pivotSim.setRawRotorPosition(pivotModel.getAngularPosition().times(PivotConstants.GEAR_RATIO));
    pivotSim.setRotorVelocity(pivotModel.getAngularVelocity().times(PivotConstants.GEAR_RATIO));

    inputs.temp = 0;
    inputs.statorCurrent = pivotModel.getCurrentDrawAmps();
    inputs.voltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.position =
        pivotMotor.getPosition().getValueAsDouble() * GlobalConstants.CONVERSION_FACTOR;
  }

  @Override
  public void changeSetpoint(double setpoint) {
    // Clamp to safe range then convert degrees to rotations for the motor controller
    positionVoltage.Position =
        MathUtil.clamp(setpoint, PivotConstants.MIN_SETPOINT, PivotConstants.MAX_SETPOINT)
            / GlobalConstants.CONVERSION_FACTOR;
  }
}
