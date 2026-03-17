package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfigs {

    static final int intakeMotorID = 4;
    static final int feederMotorID = 18;
    
    static final int feederMotorCurrentLimit = 60;

    static final SparkMaxConfig intakeConfigs =
            (SparkMaxConfig) new SparkMaxConfig().inverted(false).smartCurrentLimit(feederMotorCurrentLimit);



    static final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
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

