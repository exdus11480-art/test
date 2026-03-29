// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ExternalIntake.ExternalIntake;

import frc.robot.Subsystems.autoAim.autoAim;
import frc.robot.Subsystems.climb.Climb;
import frc.robot.Subsystems.intake.Intake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

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
        double targetX_meters = Units.inchesToMeters(492.88);
      double targetY_meters = Units.inchesToMeters(158.84);
  // Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final autoAim autoAimSubsystem = new autoAim(drivebase);

  private final Climb climb = new Climb();
  private final Shooter shooter = new Shooter(autoAimSubsystem);
  private final Intake intake = new Intake();
  private final ExternalIntake externalIntake = new ExternalIntake();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("shooter", shootCommand().withTimeout(5));
    NamedCommands.registerCommand("gyro", Commands.runOnce(() -> drivebase.zeroGyro()));

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> driverXbox.getLeftY(),
                () -> driverXbox.getLeftX())
                .withControllerRotationAxis(() -> driverXbox.getRightX())
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
    driverXbox.leftTrigger().whileTrue(intakeCommand().alongWith(open()));
    driverXbox.rightBumper().whileTrue(ejectCommand());
    driverXbox.leftBumper().whileTrue(driveAndAim());

    driverXbox.y().whileTrue(activateExternalIntake());
    driverXbox.x().whileTrue(close());
  }

  private Command driveAndAim() {

    return Commands.run(() -> {

      double[] targetData = autoAimSubsystem.getDistanceAndAngleToPoint(targetX_meters, targetY_meters);
      double targetAngle = targetData[1];

      double rotationSpeed = autoAimSubsystem.calculateRotationSpeed(targetAngle);
      // 4. נסיעה
      double xTranslation = MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND);
      double yTranslation = MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.DEADBAND);

      drivebase.drive(new Translation2d(xTranslation, yTranslation), rotationSpeed, true);
    }, drivebase, autoAimSubsystem); // חשוב להוסיף את autoAimSubsystem כדרישה (Requirement)

  }

  private Command climbUpCommand() {
    return climb.setVoltage(6);

  }

  private Command climbDownCommand() {
    return climb.setVoltage(-6);
  }

  private Command open() {
    return externalIntake.intakeProperty(0, -3);
  }

  private Command activateExternalIntake() {
    return externalIntake.intakeProperty(2, -3);
  }

  private Command close() {
    return externalIntake.intakeProperty(-1, 0);// קוראים לזה עבודה ערבית של העבודה הערבית-+
  }

  private Command shootCommand() {
      
      double targetSpeed = shooter.activateShooter();
      
      return shooter.runShooterVelocity(targetSpeed).alongWith(
          Commands.waitUntil(() -> shooter.getActualVelocity() >= targetSpeed - 2)
              .andThen(intake.runFullIntake(8.2, 8)));

  }

  // private Command shootCommand2() {
  // double targetSpeed = SmartDashboard.getNumber("Shooter Speed", 15);

  // return shooter.runShooterVelocity(targetSpeed).alongWith(
  // Commands.waitUntil(() -> shooter.getActualVelocity() >= targetSpeed - 5)
  // .andThen(intake.runFullIntake(8.2, 8)));

  // }

  private Command intakeCommand() {
    return intake.runFullIntake(8, -10.1);
  }

  private Command ejectCommand() {
    return intake.runFullIntake(-3, 3);
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

}