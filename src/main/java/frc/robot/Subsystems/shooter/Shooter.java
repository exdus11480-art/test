package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.autoAim.autoAim;
import frc.robot.Subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final autoAim autoAimSubsystem;

    private double m_targetRPS = 0;
    double targetX_meters = Units.inchesToMeters(492.88);
    double targetY_meters = Units.inchesToMeters(158.84);
    public Shooter(autoAim autoAimSubsystem) {
        this.autoAimSubsystem = autoAimSubsystem;
        shooterMotor = new TalonFX(ShooterConfigs.shooterMotorID);
        shooterMotor.getConfigurator().apply(ShooterConfigs.shooterMotorConfig);
    }

    public Command runShooterVelocity(double velocityRPS) {
        return run(() -> {
            m_targetRPS = velocityRPS;
            shooterMotor.setControl(m_velocityRequest.withVelocity(velocityRPS));
        });
    }

    public double activateShooter() {

        double[] targetData = autoAimSubsystem.getDistanceAndAngleToPoint(targetX_meters, targetY_meters);
        double distance = targetData[0];
        double RPS = SmartDashboard.getNumber("Shooter Speed", 7.55 + distance + 42.86); // Example: Speed increases
                                                                                         // with distance

        return RPS;
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

    public double getActualVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

}
