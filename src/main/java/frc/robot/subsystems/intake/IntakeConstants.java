package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {

  public class Extension {

    public static final DCMotor MOTOR = DCMotor.getFalcon500(1);

    public static final double INITIAL_SETPOINT = 0; // deg

    public static final double MIN_SETPOINT = 0; // deg
    public static final double MAX_SETPOINT = 120; // deg (fully inside)
    public static final double GEAR_RATIO = 1; // TODO: real gear ratio needed

    public static final double SUPPLY_LIMIT = 30; // Amps
    public static final double STATOR_LIMIT = 60; // Amps

    public static final TalonFXConfiguration CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true));
  }

  public class Roller {

    public static final DCMotor MOTOR = DCMotor.getFalcon500(1);
    public static final double GEAR_RATIO = 1; //TODO: real ratio

    public static final TalonFXConfiguration CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true));

    public static final double Off = 0.0; // No power
    public static final double FORWARD = 12;
    public static final double REVERSE = -12;
  }
}
