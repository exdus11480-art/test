package frc.robot.Subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    // 1. הגדרת המנוע
    private final TalonFX motor;

    public Climb() {
        // 2. אתחול המנוע עם ה-ID מה-Constants
        motor = new TalonFX(ClimbConfigs.Climb_MOTOR_ID);

    }
}