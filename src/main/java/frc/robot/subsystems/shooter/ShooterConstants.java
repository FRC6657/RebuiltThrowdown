package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {
  public static final double CONVERSION_FACTOR = 360.0; // Degrees Per Rotation

  public class PivotConstants {
    public static final double SUPPLY_LIMIT = 30; // Amps
    public static final double STATOR_LIMIT = 60; // Amps

    public static final TalonFXConfiguration CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(1)) // assuming this means 1:1 ratio?
            .withSlot0(new Slot0Configs().withKS(0).withKP(250.0).withKD(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(SUPPLY_LIMIT)
                    .withSupplyCurrentLowerTime(0));
  }

  public class FlywheelConstants {
    public static final double SUPPLY_LIMIT = 60; // Amps
    public static final double STATOR_LIMIT = 80; // Amps
    public static final double GEAR_RATIO = 1; // 1:1 Ratio

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
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(SUPPLY_LIMIT)
                    .withSupplyCurrentLowerTime(0))
            .withSlot0(new Slot0Configs().withKV(12d / (6380d / 60d)).withKP(0.5).withKD(0));
  }
}
