package frc.robot.subsystems.shooter.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class PivotIO_Real implements PivotIO {
    private TalonFX pivotMotor = new TalonFX(GlobalConstants.CAN.Shooter_Pivot.id);

    private PositionVoltage positionVoltage = new PositionVoltage(1); // TODO: Replace with real number
    public PivotIO_Real() {
        pivotMotor.getConfigurator().apply(ShooterConstants.CONFIG);

        var position = pivotMotor.getPosition();
        var temp = pivotMotor.getDeviceTemp();
        var voltage = pivotMotor.getMotorVoltage();
        var statorCurrent = pivotMotor.getStatorCurrent();

        position.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        temp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
        voltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        statorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

        pivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
    
      pivotMotor.setControl(positionVoltage);
    
      inputs.temp = pivotMotor.getDeviceTemp().getValueAsDouble();
      inputs.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
      inputs.voltage = pivotMotor.getMotorVoltage().getValueAsDouble();
      inputs.position = pivotMotor.getPosition().getValueAsDouble() * ShooterConstants.CONVERSION_FACTOR;
    }

    @Override
    public void changeSetpoint(double setpoint) {
      // Clamp to safe range then convert degrees to rotations for the motor controller
      positionVoltage.Position = 1;
      //    MathUtil.clamp(setpoint, ShooterConstants.MIN_SETPOINT, ShooterConstants.MAX_SETPOINT);
    }
}
