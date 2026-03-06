package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConfigs.intakeMotorID);
        intakeMotor.getConfigurator().apply(IntakeConfigs.intakeMotorConfig);
    }

    public Command setVoltage(double voltage) {
        return startEnd(
            () -> intakeMotor.setVoltage(voltage),
            () -> intakeMotor.setVoltage(0)
        );
    }
}