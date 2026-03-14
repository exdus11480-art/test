package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
    

public class Vision extends SubsystemBase {

    private final Limelight limelight = new Limelight("limelight");

    public Vision() {
    }


    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
