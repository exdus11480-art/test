package frc.robot.Subsystems.climb;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFX followerClimbMotor;

    public Climb() {
        climbMotor = new TalonFX(ClimbConfigs.climbMotorID);
        followerClimbMotor = new TalonFX(ClimbConfigs.followerClimbMotorID);

        climbMotor.getConfigurator().apply(ClimbConfigs.climbMotorConfig);
        followerClimbMotor.getConfigurator().apply(ClimbConfigs.climbMotorConfig);
        climbMotor.setPosition(0);
    followerClimbMotor.setControl(new Follower(climbMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public Command setVoltage(double voltage) {
        return startEnd(
                () -> climbMotor.setControl(new VoltageOut(voltage)),
                () -> climbMotor.setControl(new VoltageOut(0)));
    }


}