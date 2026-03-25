package frc.robot.Subsystems.autoAim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.swervedrive.SwerveSubsystem;

public class autoAim extends SubsystemBase {

    private final PIDController turnController = new PIDController(0.35, 0.0, 0.0);

    private final SwerveSubsystem swerve;

    public autoAim(SwerveSubsystem swerve) {
        // Initialize any necessary components here
        this.swerve = swerve;
        turnController.setTolerance(Units.degreesToRadians(2));
        turnController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double calculateRotationSpeed(double targetAngle) {
        double currentAngle = swerve.getPose().getRotation().getRadians();
                System.out.println("targetAngle = " + targetAngle);
                System.out.println("currentAngle = " + currentAngle);
        double output = - turnController.calculate(currentAngle, targetAngle);
        return MathUtil.clamp(output, -0.5, 0.5);
    }

    public double[] getDistanceAndAngleToPoint(double targetX, double targetY) {

        Pose2d robotPose = swerve.getPose();

        Translation2d targetLocation = new Translation2d(targetX, targetY);
        Translation2d robotLocation = robotPose.getTranslation();

        Translation2d relativeVector = targetLocation.minus(robotLocation);

        // חישוב המרחק (Norm נותן את אורך הוקטור)
        double distance = relativeVector.getNorm();

        // חישוב הזווית המוחלטת על המגרש (בלי להחסיר את הזווית הנוכחית של הרובוט!)
        Rotation2d targetAngle = new Rotation2d(relativeVector.getX(), relativeVector.getY());

        // החזרת התוצאות
        return new double[] { distance, targetAngle.getRadians() };
    }

    public boolean isAtTarget() {
        return turnController.atSetpoint();
    }
}
