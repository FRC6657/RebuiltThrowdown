package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.GlobalConstants;

public class IntakeIO_Sim implements IntakeIO {

  private TalonFX pivotMotor = new TalonFX(GlobalConstants.CAN.Intake_Pivot.id);
  private TalonFX rollerMotor = new TalonFX(GlobalConstants.CAN.Intake_Wheels.id);

  private VoltageOut rollerSetpoint = new VoltageOut(0);
  private PositionVoltage pivotPositionVoltage =
      new PositionVoltage(
          IntakeConstants.Pivot.INITIAL_SETPOINT / GlobalConstants.CONVERSION_FACTOR);

  // Physics simulation for the extension motor
  private DCMotorSim pivotModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, IntakeConstants.Pivot.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  // Physics simulation for the roller motor
  private DCMotorSim rollerModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, IntakeConstants.Roller.GEAR_RATIO),
          DCMotor.getFalcon500(1));

  public IntakeIO_Sim() {
    pivotMotor
        .getConfigurator()
        .apply(
            IntakeConstants.Pivot.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(false)
                    .withStatorCurrentLimitEnable(false)));
    rollerMotor
        .getConfigurator()
        .apply(
            IntakeConstants.Roller.CONFIG.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(false)
                    .withStatorCurrentLimitEnable(false)));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var pivotMotorSim = pivotMotor.getSimState();
    pivotMotorSim.setSupplyVoltage(12);
    pivotModel.setInputVoltage(pivotMotorSim.getMotorVoltage());
    pivotModel.update(1 / GlobalConstants.MAIN_LOOP_FREQUENCY);
    pivotMotorSim.setRawRotorPosition(
        pivotModel.getAngularPosition().times(IntakeConstants.Pivot.GEAR_RATIO));
    pivotMotorSim.setRotorVelocity(
        pivotModel.getAngularVelocity().times(IntakeConstants.Pivot.GEAR_RATIO));

    var rollerMotorSim = rollerMotor.getSimState();
    rollerMotorSim.setSupplyVoltage(12);
    rollerModel.setInputVoltage(rollerMotorSim.getMotorVoltage());
    rollerModel.update(1 / GlobalConstants.MAIN_LOOP_FREQUENCY);
    rollerMotorSim.setRawRotorPosition(
        rollerModel.getAngularPosition().times(IntakeConstants.Roller.GEAR_RATIO));
    rollerMotorSim.setRotorVelocity(
        rollerModel.getAngularVelocity().times(IntakeConstants.Roller.GEAR_RATIO));

    inputs.pivotPosition =
        pivotMotor.getPosition().getValueAsDouble() * GlobalConstants.CONVERSION_FACTOR;
    inputs.pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
    inputs.pivotAcceleration = pivotMotor.getAcceleration().getValueAsDouble();
    inputs.pivotTemp = 0;
    inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotStatorCurrent = pivotModel.getCurrentDrawAmps();

    inputs.rollerTemp = 0;
    inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerStatorCurrent = rollerModel.getCurrentDrawAmps();
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
