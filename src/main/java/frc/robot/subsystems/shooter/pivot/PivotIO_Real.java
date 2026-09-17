package frc.robot.subsystems.shooter.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants.PivotConstants;

public class PivotIO_Real implements PivotIO {
  private TalonFX pivotMotor = new TalonFX(GlobalConstants.CAN.Shooter_Pivot.id);

  private PositionVoltage positionVoltage =
      new PositionVoltage(PivotConstants.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);

  public PivotIO_Real() {
    pivotMotor.getConfigurator().apply(PivotConstants.CONFIG);

    var position = pivotMotor.getPosition();
    var temp = pivotMotor.getDeviceTemp();
    var voltage = pivotMotor.getMotorVoltage();
    var statorCurrent = pivotMotor.getStatorCurrent();

    position.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    temp.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY / 4d);
    voltage.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    statorCurrent.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);

    pivotMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {

    pivotMotor.setControl(positionVoltage);

    inputs.temp = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
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
