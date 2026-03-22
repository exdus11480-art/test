// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.autoAim.autoAim;
import frc.robot.Subsystems.climb.Climb;
import frc.robot.Subsystems.intake.Intake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Vision.LimelightHelpers;

import java.io.Console;
import java.io.File;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */

public class RobotContainer {

  // Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final autoAim autoAimSubsystem = new autoAim(drivebase);

  private final Climb climb = new Climb();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("shooter", shootCommand().withTimeout(5));

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> -driverXbox.getLeftX(),
                () -> -driverXbox.getLeftY())
                .withControllerRotationAxis(() -> -driverXbox.getRightX())
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true)));

    shooter.setDefaultCommand(shooter.runOnce(() -> shooter.stop()));

    // Gyro reset
    driverXbox.a().onTrue(Commands.runOnce(() -> drivebase.zeroGyro()));

    // Climb commands
    driverXbox.pov(0).whileTrue(climbUpCommand());
    driverXbox.pov(180).whileTrue(climbDownCommand());

    // Shooter, Intake and Eject
    driverXbox.rightTrigger().whileTrue(shootCommand());
    driverXbox.leftTrigger().whileTrue(intakeCommand());
    driverXbox.rightBumper().whileTrue(ejectCommand());
    driverXbox.leftBumper().whileTrue(driveAndAim());
    driverXbox.b().whileTrue((alignToAngle()));
  }

  public double[] getDistanceAndAngleToPoint(double targetX, double targetY) {
    Pose2d robotPose = drivebase.getPose(); 
    System.out.println("robot: " + robotPose.getRotation());
    
    Translation2d targetLocation = new Translation2d(targetX, targetY);
    Translation2d robotLocation = robotPose.getTranslation();
    
    Translation2d relativeVector = targetLocation.minus(robotLocation);

    double distance = relativeVector.getNorm();
    Rotation2d targetAngle = new Rotation2d(relativeVector.getX(), relativeVector.getY()).minus(robotPose.getRotation());
    System.out.println("target: " + targetAngle.getDegrees());

    // החזרת התוצאות
    return new double[] { 
        distance, 
        targetAngle.getDegrees() };
    }

  private Command driveAndAim() {
      return Commands.run(() -> {
            double angle = getDistanceAndAngleToPoint(163, 160)[1];
            double normalizedAngle = MathUtil.inputModulus(angle, 0, 360);

            if(Math.abs(angle) < 10)
            {
              angle = 0;
            }
            else
            {
              if(Math.abs(normalizedAngle) > 30)
                  normalizedAngle = 0.5;
              else
                  normalizedAngle = normalizedAngle/30/2;
            }
            System.out.println(normalizedAngle);
        drivebase.drive(new Translation2d(driverXbox.getLeftY(), driverXbox.getLeftX()), normalizedAngle, true);
      });
  }

  private Command climbUpCommand() {
    return climb.setVoltage(6);
    
  }

  private Command climbDownCommand() {
    return climb.setVoltage(-6);
  }

  private Command shootCommand() {
    double targetSpeed = SmartDashboard.getNumber("Shooter Speed", 80 );

    return shooter.runShooterVelocity(targetSpeed).alongWith(
        Commands.waitUntil(() -> shooter.getActualVelocity() >= targetSpeed - 5)
            .andThen(intake.runFullIntake(8.2, 8)));

  }

  private Command intakeCommand() {
    return intake.runFullIntake(8, -10.1);
  }

  private Command ejectCommand() {
    return intake.runFullIntake(-7, 7);
  }

  private Command alignToAngle() {
    return autoAimSubsystem.alignToAngle(157); // Example angle, replace with actual desired angle
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

}