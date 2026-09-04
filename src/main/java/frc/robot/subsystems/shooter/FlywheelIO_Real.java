package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.GlobalConstants;

public class FlywheelIO_Real implements FlywheelIO {

  private SparkMax motorOne;
  private SparkMax motorTwo;

  private PIDController pid = new PIDController(0, 0, 0);

  public FlywheelIO_Real() {
    motorOne = new SparkMax(GlobalConstants.CAN.Shooter_One.id, MotorType.kBrushless);
    motorTwo = new SparkMax(GlobalConstants.CAN.Shooter_Two.id, MotorType.kBrushless);
    motorOne.setCANTimeout(250);
    motorTwo.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.smartCurrentLimit(ShooterConstants.currentLimit);
    config.idleMode(IdleMode.kCoast);

    motorOne.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorTwo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.motorOnePosition = motorOne.getEncoder().getPosition();
    inputs.motorOneVelocity = motorOne.getEncoder().getVelocity();
    inputs.motorOneTemp = motorOne.getMotorTemperature();
    inputs.motorOneVoltage = motorOne.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.motorOneCurrent = motorOne.getOutputCurrent();

    inputs.motorTwoPosition = motorTwo.getEncoder().getPosition();
    inputs.motorTwoVelocity = motorTwo.getEncoder().getVelocity();
    inputs.motorTwoTemp = motorTwo.getMotorTemperature();
    inputs.motorTwoVoltage = motorTwo.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.motorTwoCurrent = motorTwo.getOutputCurrent();
  }
}
