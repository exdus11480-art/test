package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage; // הוספנו את זה לקוד
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final SparkMax feederRoller;
    private double targetFeederRPS = 0;
    private double targetIntakeRPS = 0; 

    private final VelocityVoltage intakeVelocityControl = new VelocityVoltage(0).withSlot(0);

    public Intake() {
        intakeMotor = new TalonFX(IntakeConfigs.intakeMotorID);
        

        intakeMotor.getConfigurator().apply(IntakeConfigs.intakeMotorConfig);
        intakeMotor.getConfigurator().apply(IntakeConfigs.intakeVelocityGains); 
        
        feederRoller = new SparkMax(IntakeConfigs.feederMotorID, SparkLowLevel.MotorType.kBrushless);
        feederRoller.configure(
                IntakeConfigs.feederMotorConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
    }

    public Command runFullIntake(double intakeVolts, double feederVolts) {
        return runEnd(
            () -> {
                intakeMotor.setVoltage(intakeVolts);
                feederRoller.setVoltage(feederVolts);
            },
            () -> {
                intakeMotor.setVoltage(0);
                feederRoller.setVoltage(0);
            }
        );
    }

    public Command runFullIntakePID(double targetIntakeRPS_Param, double targetFeederRPS_Param) {
        return runEnd(
            () -> {
                this.targetIntakeRPS = targetIntakeRPS_Param;
                this.targetFeederRPS = targetFeederRPS_Param;
                
                intakeMotor.setControl(intakeVelocityControl.withVelocity(targetIntakeRPS_Param));
                
                feederRoller.getClosedLoopController().setSetpoint(
                    targetFeederRPS_Param, 
                    SparkBase.ControlType.kVelocity, 
                    ClosedLoopSlot.kSlot0
                );
            },
            () -> {
                this.targetIntakeRPS = 0;
                this.targetFeederRPS = 0;
                intakeMotor.setVoltage(0); 
                feederRoller.setVoltage(0); 
            }
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake target rps", targetIntakeRPS);
        SmartDashboard.putNumber("intake actual rps", intakeMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("feeder target rps", targetFeederRPS);
        double actualFeederRPS = feederRoller.getEncoder().getVelocity() / 60.0;
        SmartDashboard.putNumber("feeder actual rps", actualFeederRPS);
    }
}