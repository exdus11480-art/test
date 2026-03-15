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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
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

  // Controllers
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

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
                () -> -driverXbox.getLeftY(),
                () -> -driverXbox.getLeftX())
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

    driverXbox.b().whileTrue((aa()));
  }

  private Command climbUpCommand() {
    return climb.setVoltage(6);
  }

  private Command climbDownCommand() {
    return climb.setVoltage(-6);
  }

  private Command shootCommand() {
    double targetSpeed = 80;

    return shooter.runShooterVelocity(targetSpeed).alongWith(
        Commands.waitSeconds(1.2).andThen(intake.setVoltage(10)));
  }

  private Command intakeCommand() {
    return shooter.setVoltage(4).alongWith(intake.setVoltage(-8));

  }

  private Command ejectCommand() {
    return shooter.setVoltage(-5).alongWith(intake.setVoltage(6));
  }
  private Command aa() {
    return shooter.setVoltage(12);
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

}