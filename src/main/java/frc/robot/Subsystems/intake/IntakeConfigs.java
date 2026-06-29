package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfigs {
    public static final int intakeMotorID = 4;
    public static final int feederMotorID = 18;
    static final int feederMotorCurrentLimit = 60;
    
    public static final Slot0Configs intakeVelocityGains = new Slot0Configs()
            .withKS(0.1)
            .withKV(0.11)
            .withKP(0.3) 
            .withKI(0.0)
            .withKD(0.025);

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

    public static final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();

    static {
        feederMotorConfig.inverted(false)
                         .smartCurrentLimit(feederMotorCurrentLimit);
        feederMotorConfig.closedLoop
                         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                         .pid(0.00003, 0.0, 0.001)
                         .feedForward
                            .kV(0.13926)
                            .kS(0.0)
                            .kA(0.0);
    }
}