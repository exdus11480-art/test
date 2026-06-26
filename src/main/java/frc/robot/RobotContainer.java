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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.OperatorConstants;

import frc.robot.Subsystems.autoAim.autoAim;
import frc.robot.Subsystems.climb.Climb;
import frc.robot.Subsystems.intake.FeederSubsystem;
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

  // Controllers
  final CommandXboxController driverXbox = new CommandXboxController(3);
  final CommandXboxController operatorXbox = new CommandXboxController(0); // שלט שני עבור המפעיל והטסטים

  // Subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final autoAim autoAimSubsystem = new autoAim(drivebase);
  private final Climb climb = new Climb();
  private final Shooter shooter = new Shooter(autoAimSubsystem);

private final FeederSubsystem feeder = new FeederSubsystem(); // יצירת הפידר לטסט ה-SysID

private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    // NamedCommands.registerCommand("shooter", shootCommand().withTimeout(5));
    NamedCommands.registerCommand("gyro", Commands.runOnce(() -> drivebase.zeroGyro()));

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> driverXbox.getLeftY() * 0.5,
                () -> driverXbox.getLeftX() * 0.5)
                .withControllerRotationAxis(() -> - driverXbox.getRightX() * 0.5)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true)));

    shooter.setDefaultCommand(shooter.run(shooter::stop));

    // Gyro reset
    driverXbox.a().onTrue(Commands.runOnce(() -> drivebase.zeroGyro()));

    // driverXbox.y().whileTrue(
    //     shooter.manualShootCommand()
    //         .alongWith(
    //             Commands.waitUntil(() -> shooter.getActualVelocity() >= 59) 
    //                 .andThen(intake.runFullIntakeWithPID(8.2, 2500)) 
    //         )
    // );

    // Climb commands
    // driverXbox.pov(0).whileTrue(rightclimbUpCommand());
    // driverXbox.pov(180).whileTrue(rightclimbDownCommand());
    // driverXbox.pov(90).whileTrue(leftclimbDownCommand());
    // driverXbox.pov(270).whileTrue(leftclimbUpCommand());

    // Shooter, Intake and Eject
    // driverXbox.rightTrigger().whileTrue(shootCommand());
    // driverXbox.rightBumper().whileTrue(ejectCommand());
    driverXbox.leftBumper().whileTrue(driveAndAim());
    // driverXbox.leftTrigger().whileTrue(intakeCommand());

    // =========================================================================
    // לחצני SysID עבור ה-Feeder (מבוצעים משלט המפעיל - פורט 1)
    // =========================================================================
    
    // טסטים מסוג Quasistatic (מהירות עולה לאט ובהדרגה)
    // החזקת X בשלט מפעיל + חץ למעלה = קדימה
    operatorXbox.povUp().and(operatorXbox.x())
        .whileTrue(feeder.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));

    // החזקת X בשלט מפעיל + חץ למטה = אחורה
    operatorXbox.povDown().and(operatorXbox.x())
        .whileTrue(feeder.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

    // טסטים מסוג Dynamic (מכת מתח פתאומית וישרה)
    // החזקת Y בשלט מפעיל + חץ למעלה = קדימה
    operatorXbox.povUp().and(operatorXbox.y())
        .whileTrue(feeder.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));

    // החזקת Y בשלט מפעיל + חץ למטה = אחורה
    operatorXbox.povDown().and(operatorXbox.y())
        .whileTrue(feeder.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
  }

  private Command driveAndAim() {
    return Commands.run(() -> {
      double[] targetData = autoAimSubsystem.getDistanceAndAngleToPoint(targetX_meters, targetY_meters);
      double targetAngle = targetData[1];

      double rotationSpeed = autoAimSubsystem.calculateRotationSpeed(targetAngle);
      double xTranslation = MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND);
      double yTranslation = MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.DEADBAND);

      drivebase.drive(new Translation2d(- yTranslation, - xTranslation), rotationSpeed, true);
    }, drivebase, autoAimSubsystem); 
  }

  private Command rightclimbUpCommand() {
    return climb.setVoltage(3, 0);
  }

  private Command rightclimbDownCommand() {
    return climb.setVoltage(-2, 0);
  }

  private Command leftclimbUpCommand() {
    return climb.setVoltage(3, 1);
  }

  private Command leftclimbDownCommand() {
    return climb.setVoltage(-2, 1);
  }

  // private Command shootCommand() {
  //   return shooter.runShooterVelocity(() -> shooter.activateShooter())
  //       .alongWith(
  //           Commands.waitUntil(() -> {
  //               double currentTarget = shooter.activateShooter();
  //               return shooter.getActualVelocity() >= currentTarget - 2;
  //           })
  //           .andThen(intake.runFullIntakeWithPID(8.2, 2500)) 
  //       );
  // }

  // private Command intakeCommand() {
  //   return intake.runFullIntake(8, -10.1);
  // }

  // private Command ejectCommand() {
  //   return intake.runFullIntake(-5, 5);
  // }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}