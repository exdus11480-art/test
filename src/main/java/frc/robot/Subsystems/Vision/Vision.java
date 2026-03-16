package frc.robot.Subsystems.Vision;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralClassifier;

public class Vision extends SubsystemBase {

    public Vision() {
        Limelight limelight = new Limelight("limelight");
        // Get the results
        limelight.getLatestResults().ifPresent((LimelightResults result) -> {
            for (NeuralClassifier object : result.targets_Classifier) {
                // Classifier says its a coral.
                if (object.className.equals("coral")) {
                    // Check pixel location of coral.
                    if (object.ty > 2 && object.ty < 1) {
                        // Coral is valid! do stuff!
                    }
                }
            }
        });

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
