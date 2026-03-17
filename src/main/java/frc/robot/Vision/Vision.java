package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.pipeline.NeuralClassifier;

public class Vision extends SubsystemBase {
    Limelight limelight = new Limelight("limelight");

    public Vision() {
// Basic targeting data
double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees

LimelightHelpers.setPipelineIndex("", 0);
    }

public void periodic() {
    // This method will be called once per scheduler run

}

}