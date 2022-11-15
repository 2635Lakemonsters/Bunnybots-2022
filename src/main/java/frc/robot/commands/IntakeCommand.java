// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private IntakeSubsystem m_intakeSubsystem;
  public double initialEncoderPosition;

  private boolean isButtonReleasedYet; 

  public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean isButtonReleasedYet) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isButtonReleasedYet = isButtonReleasedYet; 

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
    System.out.println("global IP = " + m_intakeSubsystem.getGlobalInitialPosition());
    if (isButtonReleasedYet == false) { // spin freely
      m_intakeSubsystem.spinIntake(-0.1);
      System.out.println("IntakeCommand.execute(): position: " + m_intakeSubsystem.getEncoderPosition());
      System.out.println("IntakeCommand.execute(): initialPosition: " + initialEncoderPosition);

    } else if (isButtonReleasedYet == true) { // correct to vertical position
      m_intakeSubsystem.spinIntake(-0.1);

      if (Math.abs(m_intakeSubsystem.getEncoderPosition()) - Math.abs(initialEncoderPosition) >= 4096/2) {
        System.out.println("initialEncoderPosition: " + initialEncoderPosition);
        System.out.println("currentEncoderPosition: " + m_intakeSubsystem.getEncoderPosition());
        end(true);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.spinIntake(0);
    System.out.println("from IntakeCommand.end()");
    System.out.println("initialEncoderPosition: " + initialEncoderPosition);
    System.out.println("currentEncoderPosition: " + m_intakeSubsystem.getEncoderPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
