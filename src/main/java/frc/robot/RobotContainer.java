// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.basic.BasicBorders.ToggleButtonBorder;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import java.util.List;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
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

  //private final SendableChooser<Command> autoChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // Operator Controller
  XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    //autoChooser = AutoBuilder.buildAutoChooser();

    //SmartDashboard.putData("Auto Chooser",autoChooser);

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

    // Launcher Position Command Groups
    ParallelCommandGroup LauncherArmCommandGroup = new ParallelCommandGroup();

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
    new JoystickButton(m_OperatorController, Button.kRightBumper.value) // USB 1 Right Bumper
      .whileTrue(m_Intake.RunIntakeCommand(0.80)); // Run intake motor FORWARD at 80% power while button held (adjust intake speed here)

  // Intake REVERSE
    new JoystickButton(m_OperatorController, Button.kLeftBumper.value) // USB 1 Left Bumper
      .whileTrue(m_Intake.RunIntakeCommand(-0.75)); // Run intake motor REVERSE at 75% power while button held (adjust intake speed here)

  // Launcher SUBWOOFER Speed
    new JoystickButton(m_OperatorController, Button.kA.value) // USB 1 - Button A
       .whileTrue(m_Launcher.RunLauncherCommand(0.60, 0.60)); // Run launcher at 60% power while button held (adjust launcher speed here)
      
  // Launcher PODIUM Speed
    new JoystickButton(m_OperatorController, Button.kB.value) // USB 1 - Button B
      .whileTrue((m_Launcher.RunLauncherCommand(0.70, 0.70))); // Run launcher at 70% power while button held (adjust launcher speed here)

  // Launcher WING Speed
    new JoystickButton(m_OperatorController, Button.kX.value) // USB 1 - Button X
      .whileTrue((m_Launcher.RunLauncherRPMCommand(1000))); // Run launcher at 1000 RPM while button held (adjust launcher speed here)

  // Launcher AMP Speed
    new JoystickButton(m_OperatorController, Button.kY.value) // USB 1 - Button Y
      .whileTrue((m_Launcher.RunLauncherCommand(0.15, 0.15))); // Run launcher at 60% power while button held (adjust launcher speed here)

  // Run Indexer
    new JoystickButton(m_driverController, Button.kRightBumper.value) // USB 0 - Right Bumper
      .whileTrue((m_Indexer.RunIndexerCommand(0.25))); // Run indexer at 50% power while button held (adjust indexer speed here)
   
  //Arm Wing Position
    new JoystickButton(m_driverController, XboxController.Button.kX.value) // USB 0 - Button X
      .onTrue(m_Arm.SetPositionCommand(50.0)); //Real position to be determined 

  //Arm Subwoofer Position
    new JoystickButton(m_driverController, XboxController.Button.kY.value) // USB 0 - Button Y
      .onTrue(m_Arm.SetPositionCommand(88.0)); //Real position to be determined

  //Arm Max Back Position
    new JoystickButton(m_driverController, XboxController.Button.kA.value) // USB 0 - Button A
      .onTrue(m_Arm.SetPositionCommand(135.0));

    
   /*   
  //Arm Podium Position
    new JoystickButton(m_OperatorController, XboxController.Button.kX.value)
      .onTrue(m_Arm.SetPositionCommand(1.0)); //Real position to be determined
  */
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  
   public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    
  }
  */
}
