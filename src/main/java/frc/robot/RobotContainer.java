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
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.OperatorConstants;


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
      double targetX_meters = Units.inchesToMeters(469.075);
     double targetY_meters = Units.inchesToMeters(158.84);

    //double targetX_meters = Units.inchesToMeters(181.555);
    //double targetY_meters = Units.inchesToMeters(158.84);

  // Controllers
  // final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandPS4Controller psController = new CommandPS4Controller(0);
  // Subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final autoAim autoAimSubsystem = new autoAim(drivebase);

  private final Climb climb = new Climb();
  private final Shooter shooter = new Shooter(autoAimSubsystem);
  private final Intake intake = new Intake();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("shooter", shootCommand().withTimeout(8));
    NamedCommands.registerCommand("gyro", Commands.runOnce(() -> drivebase.zeroGyro()));
    NamedCommands.registerCommand("aim", driveAndAim().withTimeout(1));

    NamedCommands.registerCommand("climbup", followerClimbMotorUp().withTimeout(3));
    NamedCommands.registerCommand("climbdown", followerClimbMotorDown().withTimeout(2));

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () ->  psController.getLeftY(),
                () ->  psController.getLeftX())
                .withControllerRotationAxis(() -> - psController.getRightX() * 0.6)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true)));

    shooter.setDefaultCommand(shooter.run(shooter::stop));

    // Gyro reset
    psController.cross().onTrue(Commands.runOnce(() -> drivebase.zeroGyro()));

    psController.triangle().whileTrue(
        shooter.manualShootCommand()
        .alongWith(
            Commands.waitUntil(() -> shooter.getActualVelocity() >= 55) 
            .andThen(intake.runFullIntakePID(60, 65))
        )
    );

    // Climb commands
    psController.pov(90).whileTrue(climpUp());
    psController.pov(270).whileTrue(climbDown());
    psController.pov(180).whileTrue(followerClimbMotorDown());
    psController.pov(0).whileTrue(followerClimbMotorUp());


    // Shooter, Intake and Eject
    psController.R2().whileTrue(shootCommand());
    psController.R1().whileTrue(ejectCommand());
    psController.L1().whileTrue(driveAndAim());
    psController.L2().whileTrue(intakeCommand());
  }

  private Command driveAndAim() {

    return Commands.run(() -> {

      double[] targetData = autoAimSubsystem.getDistanceAndAngleToPoint(targetX_meters, targetY_meters);
      double targetAngle = targetData[1];

      double rotationSpeed = autoAimSubsystem.calculateRotationSpeed(targetAngle);
      double xTranslation = MathUtil.applyDeadband(psController.getLeftX(), OperatorConstants.DEADBAND);
      double yTranslation = MathUtil.applyDeadband(psController.getLeftY(), OperatorConstants.DEADBAND);

      drivebase.drive(new Translation2d(- yTranslation, - xTranslation), rotationSpeed, true);
    }, drivebase, autoAimSubsystem); 
  }

  private Command followerClimbMotorUp() {
    return climb.setVoltageFollower(3.0,3.0);
  }


  private Command followerClimbMotorDown() {
    return climb.setVoltageFollower(-2.0,-2.0);
  }

    private Command climpUp() {
    return climb.setVoltage(3);

  }

  private Command climbDown() {
    return climb.setVoltage(-2);
  }



  private Command shootCommand() {
return shooter.runShooterVelocity(() -> shooter.activateShooter())
        .alongWith(
            Commands.waitUntil(() -> {
                double currentTarget = shooter.activateShooter();
                return shooter.getActualVelocity() >= currentTarget - 2;
            })
            .andThen(intake.runFullIntakePID(60, 65))
        );
  }

  private Command intakeCommand() {
    return intake.runFullIntakePID(50, -60);
  }

  private Command ejectCommand() {
    return intake.runFullIntake(-5, 5);
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("Center_blue");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

}