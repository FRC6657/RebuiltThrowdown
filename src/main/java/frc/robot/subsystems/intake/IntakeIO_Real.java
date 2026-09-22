package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.GlobalConstants;

public class IntakeIO_Real implements IntakeIO {

  private TalonFX pivotMotor = new TalonFX(GlobalConstants.CAN.Intake_Pivot.id);
  private TalonFX rollerMotor = new TalonFX(GlobalConstants.CAN.Intake_Wheels.id);

  private VoltageOut rollerSetpoint = new VoltageOut(0);
  private PositionVoltage pivotPositionVoltage =
      new PositionVoltage(
          IntakeConstants.Pivot.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);

  public IntakeIO_Real() {

    pivotMotor.getConfigurator().apply(IntakeConstants.Pivot.CONFIG);
    rollerMotor.getConfigurator().apply(IntakeConstants.Roller.CONFIG);

    var pivotPosition = pivotMotor.getPosition();
    var pivotVelocity = pivotMotor.getVelocity();
    var pivotAcceleration = pivotMotor.getAcceleration();
    var pivotTemp = pivotMotor.getDeviceTemp();
    var pivotVoltage = pivotMotor.getMotorVoltage();
    var pivotStatorCurrent = pivotMotor.getStatorCurrent();

    pivotPosition.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    pivotVelocity.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    pivotAcceleration.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    pivotTemp.setUpdateFrequency(
        GlobalConstants.MAIN_LOOP_FREQUENCY / 4d); // Temp updates less often
    pivotVoltage.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    pivotStatorCurrent.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);

    pivotMotor.optimizeBusUtilization();

    var rollerVoltage = rollerMotor.getMotorVoltage();
    var rollerStatorCurrent = rollerMotor.getSupplyCurrent();
    var rollerTemp = rollerMotor.getDeviceTemp();

    rollerTemp.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY / 4d);
    rollerVoltage.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    rollerStatorCurrent.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);

    pivotMotor.setPosition(
        IntakeConstants.Pivot.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);

    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    pivotMotor.setControl(pivotPositionVoltage);

    rollerMotor.setControl(rollerSetpoint);

    inputs.pivotPosition =
        pivotMotor.getPosition().getValueAsDouble() * GlobalConstants.CONVERSION_FACTOR;
    inputs.pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
    inputs.pivotAcceleration = pivotMotor.getAcceleration().getValueAsDouble();
    inputs.pivotTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotStatorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();

    inputs.rollerTemp = rollerMotor.getDeviceTemp().getValueAsDouble();
    inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerStatorCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void changeSetpointR(double setpoint) {}

  @Override
  public void changeSetpointP(double setpoint) {
    pivotPositionVoltage.Position =
        MathUtil.clamp(
                setpoint, IntakeConstants.Pivot.MIN_SETPOINT, IntakeConstants.Pivot.MAX_SETPOINT)
            / GlobalConstants.CONVERSION_FACTOR;
  }
}
