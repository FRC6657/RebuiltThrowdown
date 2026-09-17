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
      new PositionVoltage(IntakeConstants.Extension.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);

  public IntakeIO_Real() {

    pivotMotor.getConfigurator().apply(IntakeConstants.Extension.CONFIG);
    rollerMotor.getConfigurator().apply(IntakeConstants.Roller.CONFIG);

    var extensionPosition = pivotMotor.getPosition();
    var extensionVelocity = pivotMotor.getVelocity();
    var extensionAcceleration = pivotMotor.getAcceleration();
    var extensionTemp = pivotMotor.getDeviceTemp();
    var extensionVoltage = pivotMotor.getMotorVoltage();
    var extensionStatorCurrent = pivotMotor.getStatorCurrent();

    extensionPosition.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    extensionVelocity.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    extensionAcceleration.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    extensionTemp.setUpdateFrequency(
        GlobalConstants.MAIN_LOOP_FREQUENCY / 4d); // Temp updates less often
    extensionVoltage.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    extensionStatorCurrent.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);

    pivotMotor.optimizeBusUtilization();

    var rollerVoltage = rollerMotor.getMotorVoltage();
    var rollerStatorCurrent = rollerMotor.getSupplyCurrent();
    var rollerTemp = rollerMotor.getDeviceTemp();

    rollerTemp.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY / 4d);
    rollerVoltage.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);
    rollerStatorCurrent.setUpdateFrequency(GlobalConstants.MAIN_LOOP_FREQUENCY);

    pivotMotor.setPosition(0);

    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    pivotMotor.setControl(pivotPositionVoltage);

    rollerMotor.setControl(rollerSetpoint);

    inputs.extensionPosition =
        pivotMotor.getPosition().getValueAsDouble();
    inputs.extensionVelocity =
        pivotMotor.getVelocity().getValueAsDouble();
    inputs.extensionAcceleration =
        pivotMotor.getAcceleration().getValueAsDouble();
    inputs.extensionTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void changeSetpointR(double setpoint) {}

  @Override
  public void changeSetpointP(double setpoint) {
    pivotPositionVoltage.Position =
        MathUtil.clamp(setpoint, IntakeConstants.Extension.MIN_SETPOINT, IntakeConstants.Extension.MAX_SETPOINT)
            / GlobalConstants.CONVERSION_FACTOR;
  }
}
