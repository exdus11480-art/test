package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralClassifier;

public class Vision extends SubsystemBase {
    Limelight limelight = new Limelight("limelight");

    public Vision() {
    }

    @Override
    public void periodic() {
        limelight.getLatestResults().ifPresent((LimelightResults result) -> {
            for (NeuralClassifier object : result.targets_Classifier) {
                // Classifier says its a hub.
                if (object.className.equals("hub")) {
                    // Check pixel location of hub.
                    if (object.ty < 2 && object.ty > 1) {
                        double angle = getTx();

                    }
                }
            }
        });
    }
public double getTx() {
    return limelight.getLatestResults().map(result -> result.targets_Classifier[0].tx).orElse(0.0);
}

public double getTy() {
    return limelight.getLatestResults().map(result -> result.targets_Classifier[0].ty).orElse(0.0);

}



}
