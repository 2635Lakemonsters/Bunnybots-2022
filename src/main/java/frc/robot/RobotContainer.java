// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj.SPI;

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

  //PID and NAVX
  public static AHRS ahrs;
  public static PIDController turnController;
  public static  double rotateToAngleRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
  public static final double kP = 0.03;
  public static final double kI = 0.00;
  public static final double kD = 0.00;
  public static final double kF = 0.00;
  public static final double kToleranceDegrees = 2.0f;    
  public static final double kTargetAngleDegrees = 90.0f;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    
    try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    turnController = new PIDController(kP, kI, kD, kF);

    /**will have to fix this documentation**/
    // turnController.setInputRange(-180.0f,  180.0f);
    // turnController.setOutputRange(-1.0, 1.0);
    // turnController.setAbsoluteTolerance(kToleranceDegrees);
    // turnController.setContinuous(true);
    // turnController.disable();
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

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("AUTO", m_autoCommand);//establish default auto option

    // create other options in SmartDashBoard
    m_autoChooser.addOption("FreeSpin", m_intakeCommandFreeSpin);
    m_autoChooser.addOption("doCorrection", m_intakeCommand_doCorrection);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    return m_autoChooser.getSelected();
  }
}
