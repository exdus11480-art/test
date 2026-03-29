package frc.robot.Subsystems.ExternalIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExternalIntake extends SubsystemBase {
    private final TalonFX intakeExternalMotor;
    private final SparkMax openingMotor;

    public ExternalIntake() {
        intakeExternalMotor = new TalonFX(ExternalIntakeConfigs.intakeExternalMotorID);
        intakeExternalMotor.getConfigurator().apply(ExternalIntakeConfigs.intakeExternalMotorConfig);

        openingMotor = new SparkMax(ExternalIntakeConfigs.openingMotorID, SparkLowLevel.MotorType.kBrushless);
        openingMotor.configure(
                ExternalIntakeConfigs.openingConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    
    public Command intakeProperty(double openingMotorVolts, double ExternalintakeVolts) {
        return runEnd(
                () -> {
                    intakeExternalMotor.setVoltage(ExternalintakeVolts);
                    openingMotor.setVoltage(openingMotorVolts);
                },
                () -> {
                    intakeExternalMotor.setVoltage(0);
                    openingMotor.setVoltage(0);
                });
    }
}