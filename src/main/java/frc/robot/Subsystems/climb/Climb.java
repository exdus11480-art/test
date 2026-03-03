package frc.robot.Subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;

    public Climb() {
        climbMotor = new TalonFX(ClimbConfigs.climbMotorID);
        climbMotor.getConfigurator().apply(ClimbConfigs.climbMotorConfig);
    }

    public Command setVoltage(double voltage) {
        return startEnd(
            () -> climbMotor.setVoltage(voltage),
            () -> climbMotor.setVoltage(0)
        );
    }

    public Command climbUp() {
        return setVoltage(6);
    }

    public Command climbDown() {
        return setVoltage(-6);
    }

}