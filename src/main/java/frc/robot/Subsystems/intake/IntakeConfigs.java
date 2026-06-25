package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfigs {
    public static final int intakeMotorID = 4;
    public static final int feederMotorID = 18;
    static final int feederMotorCurrentLimit = 60;
    

    // הגדרות אינטייק (Kraken)
    public static final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12));

    // הגדרות פידר (NEO)
    public static final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
    
        static final SparkMaxConfig intakeConfigs =
                (SparkMaxConfig) new SparkMaxConfig().inverted(false).smartCurrentLimit(feederMotorCurrentLimit);

// בתוך IntakeConfigs.java
static {
    feederMotorConfig.closedLoop.pid(0.0001, 0.0, 0.0); // הגדרת ה-PID כאן
    feederMotorConfig.closedLoop.outputRange(-1, 1);    // הגדרת טווח מתח
}


}