package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final SparkMax feederRoller;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConfigs.intakeMotorID);
        intakeMotor.getConfigurator().apply(IntakeConfigs.intakeMotorConfig);
        
        feederRoller = new SparkMax(IntakeConfigs.feederMotorID, SparkLowLevel.MotorType.kBrushed);
        feederRoller.configure(
                IntakeConfigs.intakeConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command setVoltage(double voltage) {
        return startEnd(
                () -> intakeMotor.setVoltage(voltage),
                () -> intakeMotor.setVoltage(0));
    }

    public Command setVoltagefeeder(double voltage) {
        return runEnd(() -> feederRoller.setVoltage(voltage), () -> feederRoller.setVoltage(0));
    }
}