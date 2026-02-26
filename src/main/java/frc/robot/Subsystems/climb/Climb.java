package frc.robot.Subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;

    public Climb() {
        climbMotor = new TalonFX(ClimbConfigs.climbMotorID);
        climbMotor.setInverted(ClimbConfigs.climbMotorInverted);
    }

    public Command setVoltage(double voltage) {
        return startEnd(() -> climbMotor.setVoltage(voltage), () -> climbMotor.setVoltage(0));
    }

    public Command eject() {
        return startEnd(() -> climbMotor.setVoltage(-6), () -> climbMotor.setVoltage(0));
    }

    public Command testing() {
        return startEnd(() -> climbMotor.set(1), () -> climbMotor.set(0));
    }
}
