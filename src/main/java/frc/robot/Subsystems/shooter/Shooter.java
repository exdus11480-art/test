package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    private double m_targetRPS = 0;

    public Shooter() {
        shooterMotor = new TalonFX(ShooterConfigs.shooterMotorID);
        shooterMotor.getConfigurator().apply(ShooterConfigs.shooterMotorConfig);
    }

    public Command runShooterVelocity(double velocityRPS) {
        return run(() -> {
            m_targetRPS = velocityRPS; 
            shooterMotor.setControl(m_velocityRequest.withVelocity(velocityRPS));
        });
    }

    public Command setVoltage(double voltage) {
        return startEnd(
            () -> {
                m_targetRPS = 0;
                shooterMotor.setVoltage(voltage);
            },
            () -> shooterMotor.setVoltage(0)
        );
    }

    public void stop() {
        m_targetRPS = 0;
        shooterMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Actual RPS", shooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Target RPS", m_targetRPS);
        SmartDashboard.putNumber("Shooter/Velocity Error", m_targetRPS - shooterMotor.getVelocity().getValueAsDouble());
    }


    // פונקציה שמחזירה את המהירות הנוכחית ב-RPS
public double getActualVelocity() {
    // אם את משתמשת ב-TalonFX (Phoenix 6):
    return shooterMotor.getVelocity().getValueAsDouble();
}

}


