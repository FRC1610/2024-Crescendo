// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoCommandConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbAxisCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.GoalEndState;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_Intake = new Intake();
  private final Arm m_Arm = new Arm();
  private final Launcher m_Launcher = new Launcher();
  private final Indexer m_Indexer = new Indexer();
  private final Climber m_Climber = new Climber();

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // Operator Controller
  XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    double tx = LimelightHelpers.getTX("");

    // NAMED COMMANDS
    //NamedCommands.registerCommand("SubwooferPosition", SubwooferCommandGroup());
    NamedCommands.registerCommand("SubShootNote", AutoSubShootCommand());
    NamedCommands.registerCommand("PodiumShootNote", AutoPodiumShootCommand());
    //NamedCommands.registerCommand("Shoot:", m_Indexer.RunIndexerCommand(.5).withTimeout(0.25));
    //NamedCommands.registerCommand("StopIndexer", m_Indexer.StopIndexerCommand());
    //NamedCommands.registerCommand("StopLauncher", m_Launcher.StopLauncherCommand());
    NamedCommands.registerCommand("IntakeNote", IntakeCommandGroup());
    NamedCommands.registerCommand("ArmDown", m_Arm.SetPositionCommand(ArmConstants.kArmMin).withTimeout(0.5));

    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    // Intake
    m_Intake.setDefaultCommand(m_Intake.StopIntakeCommand());

    // Launcher
    m_Launcher.setDefaultCommand(m_Launcher.StopLauncherCommand());

    // Indexer
    m_Indexer.setDefaultCommand(m_Indexer.StopIndexerCommand());

    // Arm
    m_Arm.setDefaultCommand(m_Arm.RestArmCommand());

    // Climber
    m_Climber.setDefaultCommand(m_Climber.StopClimberCommand());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {
    // Swerve
    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // Intake FORWARD
    // new JoystickButton(m_OperatorController, Button.kRightBumper.value) // USB 1
    // Right Bumper
    // .whileTrue(m_Intake.RunIntakeCommand(IntakeConstants.kIntakeSpeed)); // Run
    // intake motor FORWARD at 80% power while button held (adjust intake speed
    // here)

    // Intake REVERSE
    new JoystickButton(m_OperatorController, Button.kLeftBumper.value) // USB 1 Left Bumper
        .whileTrue(m_Intake.RunIntakeCommand(-0.75)); // Run intake motor REVERSE at 75% power while button held (adjust intake speed here)

    // Run Indexer
    new JoystickButton(m_driverController, Button.kRightBumper.value) // USB 0 - Right Bumper
        .whileTrue(m_Indexer.RunIndexerCommand(IndexerConstants.kIndexerSpeed)); // Run indexer while button held

    // Arm Wing Position
    // new JoystickButton(m_driverController, XboxController.Button.kX.value) // USB
    // 0 - Button X
    // .onTrue(m_Arm.SetPositionCommand(ArmConstants.kArmWingPosition));

    // Arm Subwoofer Position
    // new JoystickButton(m_driverController, XboxController.Button.kY.value) // USB
    // 0 - Button Y
    // .onTrue(m_Arm.SetPositionCommand(ArmConstants.kArmSubwooferPosition));

    // Arm Max Back Position
    // new JoystickButton(m_driverController, XboxController.Button.kA.value) // USB
    // 0 - Button A
    // .onTrue(m_Arm.SetPositionCommand(ArmConstants.kArmMax));

    // Climber
    //new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //    .whileTrue(m_Climber.RunClimberCommand(0.25, 0.25))
    //    .whileFalse(m_Climber.StopClimberCommand());

    // Intake Position and Run Intake
    new JoystickButton(m_OperatorController, XboxController.Button.kRightBumper.value) // USB 1 - Button Right Bumper
        // .onTrue(IntakeCommandGroup())
        // .onFalse(m_Intake.StopIntakeCommand());
        .onTrue(IntakeCommandGroup())
        .onFalse(m_Indexer.StopIndexerCommand())
        .onFalse(m_Intake.StopIntakeCommand());

    // Subwoofer Position and Speed
    new JoystickButton(m_OperatorController, XboxController.Button.kA.value) // USB 1 - Button A
        .onTrue(SubwooferCommandGroup())
        .onFalse(m_Launcher.StopLauncherCommand());

    // Podium Position and Speed
    new JoystickButton(m_OperatorController, XboxController.Button.kB.value) // USB 1 - Button B
        .onTrue(PodiumCommandGroup())
        .onFalse(m_Launcher.StopLauncherCommand());

    // Wing Position and Speed
    new JoystickButton(m_OperatorController, XboxController.Button.kY.value) // USB 1 - Button Y
        .onTrue(WingCommandGroup())
        .onFalse(m_Launcher.StopLauncherCommand());

    // Arm Down Position
    new JoystickButton(m_OperatorController, XboxController.Button.kX.value) // USB 1 - Button X
        .onTrue(m_Arm.SetPositionCommand(ArmConstants.kArmMin));

    // Source Position
    new JoystickButton(m_OperatorController, XboxController.Button.kStart.value) // USB 1 - Button Start
        .onTrue(SourceCommandGroup())
        .onFalse(m_Indexer.StopIndexerCommand())
        .onFalse(m_Launcher.StopLauncherCommand());

    // Amp Position
    new JoystickButton(m_OperatorController, XboxController.Button.kBack.value) // USB 1 - Button Start
        .onTrue(AmpCommandGroup())
        .onFalse(m_Launcher.StopLauncherCommand());
  
    // Reset Gyro
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading()
        ));

    }

  /**
   * Robot container teleop init called in Robot.java
   */
    public void teleopInit() {
        new ClimbAxisCommand(m_Climber, () -> m_OperatorController.getLeftY(), () -> m_OperatorController.getRightY()).schedule();
        m_robotDrive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(m_robotDrive.getHeading())));  //346 was here
    }

    private void SetRumble (double RumbleSpeed){
        m_OperatorController.setRumble(RumbleType.kBothRumble, RumbleSpeed);
        m_driverController.setRumble(RumbleType.kBothRumble, RumbleSpeed);
    }

    public Command RumbleCommand (double RumbleSpeed){
        return Commands.startEnd(() -> SetRumble(RumbleSpeed), () -> SetRumble(0));
    }

  // TELEOP COMMAND GROUPS

  // Puts arm in intake position, runs intake, runs Indexer until Note detected
  public Command IntakeCommandGroup() {
    return new ParallelCommandGroup(
        m_Intake.RunIntakeCommand(IntakeConstants.kIntakeSpeed).until(m_Indexer::hasNote).andThen(RumbleCommand(IndexerConstants.kIndexerRumbleSpeed).withTimeout(0.75) ),
        m_Indexer.IntakeNoteCommand(),
        m_Arm.SetPositionCommand(ArmConstants.kArmIntakePosition));
  }

  // Puts arm in Subwoofer position, runs flywheels
  public Command SubwooferCommandGroup() {
    return new ParallelCommandGroup(
        m_Launcher.RunLauncherCommand(LauncherConstants.kLauncherSubwooferSpeed,
            LauncherConstants.kLauncherSubwooferSpeed),
        m_Arm.SetPositionCommand(ArmConstants.kArmSubwooferPosition).until(m_Arm::armAtSetpoint));
  }

  // Puts arm in Podium position, runs flywheels
  public Command PodiumCommandGroup() {
    return new ParallelCommandGroup(
        m_Launcher.RunLauncherCommand(LauncherConstants.kLauncherPodiumSpeed, LauncherConstants.kLauncherPodiumSpeed),
        m_Arm.SetPositionCommand(ArmConstants.kArmPodiumPosition));
  }

  // Puts arm in Wing position, runs flywheels
  public Command WingCommandGroup() {
    return new ParallelCommandGroup(
        m_Launcher.RunLauncherCommand(LauncherConstants.kLauncherWingSpeed, LauncherConstants.kLauncherWingSpeed),
        m_Arm.SetPositionCommand(ArmConstants.kArmWingPosition));
  }

  // Puts arm in Source position, runs flywheels and indexer in reverse
  public Command SourceCommandGroup() {
    return new ParallelCommandGroup(
        m_Launcher.RunLauncherCommand(LauncherConstants.kLauncherSourceSpeed, LauncherConstants.kLauncherSourceSpeed),
        m_Indexer.RunIndexerCommand(IndexerConstants.kIndexerSourceSpeed),
        m_Arm.SetPositionCommand(ArmConstants.kArmSourcePosition));
  }

// Puts arm in Amp position, runs flywheels and indexer in reverse
  public Command AmpCommandGroup() {
    return new ParallelCommandGroup(
        m_Launcher.RunLauncherCommand(LauncherConstants.kLauncherAmpSpeed, LauncherConstants.kLauncherAmpSpeed),
        m_Arm.SetPositionCommand(ArmConstants.kArmAmpPosition));
  }

  // AUTONOMOUS COMMANDS

  // Puts arm in Subwoofer position and runs flywheels in parallel, then runs Indexer after timeout
  public Command AutoSubShootCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            m_Launcher
            .RunLauncherCommand(LauncherConstants.kLauncherSubwooferSpeed, LauncherConstants.kLauncherSubwooferSpeed)
            .withTimeout(AutoCommandConstants.kAutoSubwooferTimeout),
            m_Arm
            .SetPositionCommand(ArmConstants.kArmSubwooferPosition)
            .until(m_Arm::armAtSetpoint)),
        m_Indexer
            .RunIndexerCommand(IndexerConstants.kIndexerSpeed)
            .withTimeout(AutoCommandConstants.kAutoIndexerTimeout));
  }

  // Puts arm in Podium position and runs flywheels in parallel, then runs Indexer after timeout
  public Command AutoPodiumShootCommand() {
    return new SequentialCommandGroup(
        m_Arm
            .SetPositionCommand(ArmConstants.kArmPodiumPosition)
            .until(m_Arm::armAtSetpoint),
        m_Launcher
            .RunLauncherCommand(LauncherConstants.kLauncherPodiumSpeed, LauncherConstants.kLauncherPodiumSpeed)
            .withTimeout(AutoCommandConstants.kAutoPodiumTimeout),
        m_Indexer
            .RunIndexerCommand(IndexerConstants.kIndexerSpeed)
            .withTimeout(AutoCommandConstants.kAutoIndexerTimeout));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }

  public void periodic(){}

}