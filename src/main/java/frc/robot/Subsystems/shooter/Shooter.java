package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor;

    public Shooter() {
        shooterMotor = new TalonFX(ShooterConfigs.shooterMotorID);
        shooterMotor.getConfigurator().apply(ShooterConfigs.shooterMotorConfig);
    }

    public Command setVoltage(double voltage) {
        return startEnd(
            () -> shooterMotor.setVoltage(voltage),
            () -> shooterMotor.setVoltage(0)
        );
    }

}