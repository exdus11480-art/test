package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConfigs {

    static final int shooterMotorID = 3;

    public static final Slot0Configs shooterVelocityGains = new Slot0Configs()
            .withKS(0.1)
            .withKV(0.12)
            .withKP(0.42) 
            .withKI(0.0)
            .withKD(0.01);

    public static final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration()
            .withSlot0(shooterVelocityGains)
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(65)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(85)
                    .withStatorCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12));
}