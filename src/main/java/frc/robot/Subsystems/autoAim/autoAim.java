package frc.robot.Subsystems.autoAim;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.swervedrive.SwerveSubsystem;

public class autoAim extends SubsystemBase {
  
private final PIDController turnController = new PIDController(0.08, 0.0, 0.0);

private final SwerveSubsystem swerve; 

    public autoAim(SwerveSubsystem swerve) {
        // Initialize any necessary components here
        this.swerve = swerve;
        turnController.setTolerance(2.0);
        turnController.enableContinuousInput(-180, 180);
    }


  public Command alignToAngle(double targetAngleDegrees) {

  final double finalTarget = 10; // Example target angle, replace with actual desired angle


    return run(() -> {
      double currentAngle = swerve.getHeading().getDegrees();
      double rotationSpeed = turnController.calculate(currentAngle, finalTarget);

      swerve.swerveDrive.setChassisSpeeds(
          new ChassisSpeeds(0, 0, rotationSpeed));
    })
        .until(turnController::atSetpoint) 
        .finallyDo(() -> swerve.swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0))); 

  }
}