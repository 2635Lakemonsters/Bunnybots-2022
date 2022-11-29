// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousGroup;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // SUBSYSTEMS
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrainSubsystem m_drivetrainSubsystem = new DriveTrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevatorSubsytem = new ElevatorSubsystem();
  //shuffleboard auto chooser
  private SendableChooser<CommandGroupBase> m_autoChooser;

  // COMMANDS
    //these command declarations don't mean anything, they aren't called in robot container
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrainCommand m_driveTrainCommand = new DriveTrainCommand(m_drivetrainSubsystem, 0);
    //these mean things, as they are called in robot container off different events
  private final IntakeCommand m_intakeCommandFreeSpin = new IntakeCommand(m_intakeSubsystem, false);
  private final IntakeCommand m_intakeCommand_doCorrection = new IntakeCommand(m_intakeSubsystem, true);
    //elevator commands, half and full and down
  private final ElevatorUpCommand m_elevHalfUpCommand = new ElevatorUpCommand(m_elevatorSubsytem, true);
  private final ElevatorUpCommand m_elevFullUpCommand = new ElevatorUpCommand(m_elevatorSubsytem, false);
  private final ElevatorDownCommand m_elevDownCommand = new ElevatorDownCommand(m_elevatorSubsytem);

  //Auto Sequences
  private final AutonomousGroup m_autonomousGroup = new AutonomousGroup(m_drivetrainSubsystem,  m_intakeSubsystem);

  // JOYSTICKS
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button intakeButton = new JoystickButton(rightJoystick, Constants.R_SPIN_INTAKE_FORWARD_BUTTON);
    Button elevHalfUpButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_HALF_UP_BUTTON);
    Button elevFullUpButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_FULL_UP_BUTTON);
    Button elevDownButton = new JoystickButton(rightJoystick, Constants.ELEVATOR_DOWN_BUTTON);
    //free spins when intakeButton is held, corrects with a slower speed when intakeButton is released
    intakeButton.whenHeld(m_intakeCommandFreeSpin);
    intakeButton.whenReleased(m_intakeCommand_doCorrection);
    //does elevator commands depending on what button pressed
    elevHalfUpButton.whenPressed(m_elevHalfUpCommand);
    elevFullUpButton.whenPressed(m_elevFullUpCommand);
    elevDownButton.whenPressed(m_elevDownCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    m_autoChooser = new SendableChooser<>();
    m_autoChooser.addOption("Drive Straight", m_autonomousGroup);
    SmartDashboard.putData("AutoMode", m_autoChooser);
    
    // return m_autoChooser.getSelected();
    return m_autonomousGroup;
  }
}
