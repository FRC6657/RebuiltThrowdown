package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension.PivotSetpoint;

public class IntakeIO_Real implements IntakeIO {

  private TalonFX pivotMotor = new TalonFX(GlobalConstants.CAN.Intake_Pivot.id);
  private TalonFX rollerMotor = new TalonFX(GlobalConstants.CAN.Intake_Wheels.id);

  private VoltageOut rollerSetpoint = new VoltageOut(0);
  private PivotSetpoint pivotSetpoint = PivotSetpoint.RETRACTED_FAST;
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  public IntakeIO_Real() {

    pivotMotor.getConfigurator().apply(IntakeConstants.Extension.CONFIG);
    rollerMotor.getConfigurator().apply(IntakeConstants.Roller.CONFIG);

    var extensionPosition = pivotMotor.getPosition();
    var extensionVelocity = pivotMotor.getVelocity();
    var extensionAcceleration = pivotMotor.getAcceleration();
    var extensionTemp = pivotMotor.getDeviceTemp();
    var extensionVoltage = pivotMotor.getMotorVoltage();
    var extensionStatorCurrent = pivotMotor.getStatorCurrent();

    extensionPosition.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    extensionVelocity.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    extensionAcceleration.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    extensionTemp.setUpdateFrequency(
        GlobalConstants.mainLoopFrequency / 4d); // Temp updates less often
    extensionVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    extensionStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    pivotMotor.optimizeBusUtilization();

    var rollerVoltage = rollerMotor.getMotorVoltage();
    var rollerStatorCurrent = rollerMotor.getSupplyCurrent();
    var rollerTemp = rollerMotor.getDeviceTemp();

    rollerTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
    rollerVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
    rollerStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

    pivotMotor.setPosition(0);

    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    pivotMotor.setControl(pivotRequest.withPosition(pivotSetpoint.position));

    rollerMotor.setControl(rollerSetpoint);

    inputs.extensionPosition =
        pivotMotor.getPosition().getValueAsDouble() * IntakeConstants.Extension.CONVERSION_FACTOR;
    inputs.extensionVelocity =
        pivotMotor.getVelocity().getValueAsDouble() * IntakeConstants.Extension.CONVERSION_FACTOR;
    inputs.extensionAcceleration =
        pivotMotor.getAcceleration().getValueAsDouble()
            * IntakeConstants.Extension.CONVERSION_FACTOR;
    inputs.extensionTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(PivotSetpoint setpoint) {}

  @Override
  public void changeSetpoint(double setpoint) {
    rollerSetpoint.Output = setpoint;
  }

  @Override
  public boolean atSetpoint() {
    //   return MathUtil.isNear(
    //     extensionMotor.getGoal().position,
    //       extensionMotor.getPosition().getValueAsDouble()
    //           * IntakeConstants.Extension.CONVERSION_FACTOR,
    //       IntakeConstants.Extension.POSITION_TOLERANCE);
    return true; // FIX
  }
}
