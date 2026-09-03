package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;

public class IntakeIO_Real {
    
    private TalonFX extensionMotor = new TalonFX(GlobalConstants.CAN.Intake_Extension.id);
    private TalonFX rollerMotor = new TalonFX(GlobalConstants.CAN.Intake_Wheels.id);

    private VoltageOut rollerSetpoint = new VoltageOut(0);
    private ExtensionSetpoint extensionSetpoint = ExtensionSetpoint.RETRACTED_FAST;

    private ProfiledPIDController extensionPID =
        new ProfiledPIDController(
            1, 0, 0, new Constraints(extensionSetpoint.velocity, extensionSetpoint.velocity * 2));

    public IntakeIO_Real() {

        extensionMotor.getConfigurator().apply(IntakeConstants.Extension.CONFIG);
        rollerMotor.getConfigurator().apply(IntakeConstants.Roller.CONFIG);

        var extensionPosition = extensionMotor.getPosition();
        var extensionVelocity = extensionMotor.getVelocity();
        var extensionAcceleration = extensionMotor.getAcceleration();
        var extensionTemp = extensionMotor.getDeviceTemp();
        var extensionVoltage = extensionMotor.getMotorVoltage();
        var extensionStatorCurrent = extensionMotor.getStatorCurrent();

        extensionPosition.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extensionVelocity.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extensionAcceleration.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extensionTemp.setUpdateFrequency(
            GlobalConstants.mainLoopFrequency / 4d); //Temp updates less often
        extensionVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        extensionStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

        extensionMotor.optimizeBusUtilization();

        var rollerVoltage = rollerMotor.getMotorVoltage();
        var rollerStatorCurrent = rollerMotor.getSupplyCurrent();
        var rollerTemp = rollerMotor.getDeviceTemp();

        rollerTemp.setUpdateFrequency(GlobalConstants.mainLoopFrequency / 4d);
        rollerVoltage.setUpdateFrequency(GlobalConstants.mainLoopFrequency);
        rollerStatorCurrent.setUpdateFrequency(GlobalConstants.mainLoopFrequency);

        extensionMotor.setPosition(0);

        rollerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        extensionMotor.setControl(
            new VoltageOut(
                extensionPID.calculate(
                    extensionMotor.getPosition().getValueAsDouble()
                        * IntakeConstants.Extension.CONVERSION_FACTOR)));

        rollerMotor.setControl(rollerSetpoint);

        inputs.extensionPosition =
            extensionMotor.getPosition().getValueAsDouble()
                * IntakeConstants.Extension.CONVERSION_FACTOR;
        inputs.extensionVelocity =
            extensionMotor.getVelocity().getValueAsDouble()
                * IntakeConstants.Extension.CONVERSION_FACTOR;
        inputs.extensionAcceleration =
            extensionMotor.getAcceleration().getValueAsDouble()
                * IntakeConstants.Extension.CONVERSION_FACTOR;
        inputs.extensionTemp = extensionMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void changeSetpoint(ExtensionSetpoint setpoint) {

        extensionPID.reset(
            extensionMotor.getPosition().getValueAsDouble()
                * IntakeConstants.Extension.CONVERSION_FACTOR,
            extensionMotor.getVelocity().getValueAsDouble()
                * IntakeConstants.Extension.CONVERSION_FACTOR);

        extensionPID.setConstraints(new Constraints(setpoint.velocity, setpoint.acceleration));
        extensionPID.setGoal(setpoint.position);
    }

    @Override
    public void changeSetpoint(double setpoint) {
        rollerSetpoint.Output = setpoint;
    }

    @Override
    public boolean atSetpoint() {
        return MathUtil.isNear(
            extensionMotor.getGoal().position, 
            extensionMotor.getPosition().getValueAsDouble()
                * IntakeConstants.Extension.CONVERSION_FACTOR,
            IntakeConstants.Extension.POSITION_TOLERANCE);
    }
}

