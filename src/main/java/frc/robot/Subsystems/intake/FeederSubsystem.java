package frc.robot.Subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*; 

public class FeederSubsystem extends SubsystemBase {
    // הגדרת המנוע (לפי הספריה החדשה של REV)
    private final SparkMax m_motor = new SparkMax(1, MotorType.kBrushless); 

    // הגדרת שגרת ה-SysID 
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // 1. נתינת מתח למנוע
            (volts) -> m_motor.setVoltage(volts.in(Volts)),
            // 2. רישום נתונים ללוג
            log -> {
                log.motor("feeder-motor")
                   .voltage(Volts.of(m_motor.get() * 12.0));
                log.motor("feeder-motor")
                .voltage(Volts.of(m_motor.get() * 12.0))
                .angularPosition(Rotations.of(m_motor.getEncoder().getPosition()))
                .angularVelocity(RotationsPerSecond.of(m_motor.getEncoder().getVelocity() / 60.0));
            },
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}