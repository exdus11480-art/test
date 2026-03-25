
package frc.robot.Subsystems.ExternalIntake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ExternalIntakeConfigs {

    static final int intakeExternalMotorID = 4;
    static final int openingMotorID = 20;
    
    static final int openingMotorCurrentLimit = 30;

    static final SparkMaxConfig openingConfigs =
            (SparkMaxConfig) new SparkMaxConfig().inverted(true).smartCurrentLimit(openingMotorCurrentLimit);


    static final TalonFXConfiguration intakeExternalMotorConfig = new TalonFXConfiguration()
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
} 

