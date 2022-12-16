// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private IntakeSubsystem m_intakeSubsystem;
  private ElevatorSubsystem m_elevatorSubsystem;
  public double initialEncoderPosition;

  private boolean isButtonReleasedYet; 

  public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean isButtonReleasedYet, ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isButtonReleasedYet = isButtonReleasedYet; 
    //if isButtonReleasedYet 
    m_elevatorSubsystem = elevatorSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialEncoderPosition = m_intakeSubsystem.getEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("global IP = " + m_intakeSubsystem.getGlobalInitialPosition());
    double currentEP = m_intakeSubsystem.getEncoderPosition();
    if (isButtonReleasedYet == false) { // spin freely
      m_intakeSubsystem.spinIntake(Constants.FREE_SPIN_SPEED);
      // System.out.println("IntakeCommand.execute(): position: " + m_intakeSubsystem.getEncoderPosition());
      // System.out.println("IntakeCommand.execute(): initialPosition: " + initialEncoderPosition);

    } else if (isButtonReleasedYet == true) { // correct to vertical position
      m_intakeSubsystem.spinIntake(Constants.CORRECTION_SPEED);

      if (
        (
          2048 - Math.abs(
            Math.abs(currentEP) - Math.abs(m_intakeSubsystem.getGlobalInitialPosition())
          ) % 2048
        ) 
        >= 
        (2048-250) //
      ) {
        System.out.println("initialEncoderPosition: " + initialEncoderPosition);
        System.out.println("currentEncoderPosition: " + m_intakeSubsystem.getEncoderPosition());
        System.out.println("Math.abs(currentEP): " + Math.abs(currentEP));
        System.out.println("Math.abs(m_intakeSubsystem.getGlobalInitialPosition(): " + Math.abs(m_intakeSubsystem.getGlobalInitialPosition());
        System.out.println("Math.abs(delta): "+ (Math.abs(
            Math.abs(currentEP) - Math.abs(m_intakeSubsystem.getGlobalInitialPosition())
          ) % 2048)
        );
        System.out.println("position passed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        end(true);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("IntakeCommand ENDED====================================");
    m_intakeSubsystem.spinIntake(0);
    if (interrupted == true) {
      m_intakeSubsystem.spinIntake(0);
    }
    //System.out.println("from IntakeCommand.end()");
    //System.out.println("initialEncoderPosition: " + initialEncoderPosition);
    //System.out.println("currentEncoderPosition: " + m_intakeSubsystem.getEncoderPosition());
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_elevatorSubsystem.isAtBottom()){
    //   return false;

    // }
    return false;
  }
}

