// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.GoalEndState;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.utils.GamepadUtils;

//import java.time.Instant;
//import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final SendableChooser<Command> autoChooser;
 // private SendableChooser<Command> autoChooser  = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_codriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //named commands
    NamedCommands.registerCommand("retract", m_intake.retract());
    NamedCommands.registerCommand("stop feed", new RunCommand(() -> m_intake.setfeedPower(0), m_intake));
    NamedCommands.registerCommand("Shoot fwd", m_intake.feedbackLauncher(m_launcher).withTimeout(1.1));
   // NamedCommands.registerCommand("setfeedPower-1", new InstantCommand(() -> m_intake.setfeedPower(-1),m_intake));
    NamedCommands.registerCommand("Stop Shoot", new InstantCommand(() -> m_launcher.stopLauncher(), m_launcher));
   // NamedCommands.registerCommand("stopLauncher", new InstantCommand(() -> m_launcher.stopLauncher(), m_launcher));
    NamedCommands.registerCommand("Intake floor",  new RunCommand(() -> m_intake.setPower(1), m_intake).withTimeout(1.2));
    NamedCommands.registerCommand("Intake Shooter",  new RunCommand(() -> m_launcher.setIntake(), m_launcher).withTimeout(1.2));
    NamedCommands.registerCommand("Stop shoot intake", new InstantCommand(() -> m_launcher.stopLauncher(), m_launcher));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> m_intake.setPower(0), m_intake));
    NamedCommands.registerCommand("Stop", new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive).withTimeout(.01));
    NamedCommands.registerCommand("setScoringPosition", new InstantCommand(() -> m_arm.shoot() , m_arm));
    NamedCommands.registerCommand("setScoringPositionFar", new InstantCommand(() -> m_arm.shootfar() , m_arm));
   
    // Configure the button bindings
    configureButtonBindings();
   
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    false,
                    false),
            m_robotDrive));

    // set the arm subsystem to run the "runAutomatic" function continuously when no other command
    // is running
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("2 piece", new PathPlannerAuto("2 piece"));
    SmartDashboard.putData("4 piece", new PathPlannerAuto("4 piece"));
    SmartDashboard.putData("wing", new PathPlannerAuto("wing 2"));
    SmartDashboard.putData("centernomove", new PathPlannerAuto("centernomove"));
    SmartDashboard.putData("sourcenomove", new PathPlannerAuto("sourcenomove"));
    
    // button to put swerve modules in an "x" configuration to hold position
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // set up arm preset positions
   /* new JoystickButton(m_codriverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition))); */
    new JoystickButton(m_codriverController, XboxController.Button.kLeftBumper.value)
        .onTrue(
        new SequentialCommandGroup(   
        m_intake.retract().withTimeout(.5),
       // new RunCommand(()-> m_intake.stopfeedPower(0)),    
        m_intake.feedbackLauncher(m_launcher) )
        ) ;
    
        new Trigger(
            () ->
                m_codriverController.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    
        new JoystickButton(m_codriverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

        new JoystickButton(m_codriverController, XboxController.Button.kLeftStick.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringFarPosition)));

        new JoystickButton(m_codriverController, XboxController.Button.kRightStick.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kClosefrontPosition)));
    
    
        new JoystickButton(m_codriverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringFrontPosition)));

    

    // intake controls (run while button is held down, run retract command once when the button is
    // released)
    new Trigger(
            () ->
                m_codriverController.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(

            new ParallelCommandGroup(
                new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower) , m_intake),
                new RunCommand(() -> m_launcher.setIntake())
             )
            
            )
        .onFalse( new RunCommand(() -> m_intake.setPower(0) , m_intake) 
            );

    new JoystickButton(m_codriverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-9) , m_intake) 
            )
        .onFalse( new RunCommand(() -> m_intake.setPower(0) , m_intake) );

    new JoystickButton(m_codriverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_launcher.setIntake()));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(m_codriverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    new JoystickButton(m_codriverController, XboxController.Button.kA.value)
        .onTrue(m_intake.feedLauncher(m_launcher));

    new JoystickButton(m_codriverController, XboxController.Button.kX.value)
        .onTrue(m_intake.feedAMP(m_launcher));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();


  }
}
