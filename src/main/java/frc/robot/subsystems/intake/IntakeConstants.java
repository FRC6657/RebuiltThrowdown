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

    public static final double INITIAL_SETPOINT = 0; // in

    public static final double MIN_SETPOINT = 0; // in (fully retracted)
    public static final double MAX_SETPOINT = 12.375; // in (fully extended)
    public static final double SHUFFLE_SETPOINT = 5.0; // in (partially extended)
    public static final double POSITION_TOLERANCE = 0.5; // in

    public static final double SHUFFLE_PERIOD = 1.5;

    public static final double GEAR_RATIO = (5d / 1d) * (36d / 28d) * (28d / 15d);

    public static final double CONVERSION_FACTOR = Math.PI; // Linear Inches Per Rotation

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

    public static enum ExtensionSetpoint {
      RETRACTED_SLOW(MIN_SETPOINT + 1, 5, 40),
      RETRACTED_FAST(MIN_SETPOINT + 1, 80, 160),
      EXTENDED_SLOW(MAX_SETPOINT, 5, 40);
    }
  }
}
