// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private IntakeSubsystem m_intakeSubsystem;
  private double initialEncoderPosition;
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    m_intakeSubsystem.spinIntake(0.4);
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
    System.out.println("initialEncoderPosition: " + initialEncoderPosition);
    System.out.println("currentEncoderPosition: " + m_intakeSubsystem.getEncoderPosition());
    if (m_intakeSubsystem.getEncoderPosition() - initialEncoderPosition >= 4096/4) {
      // System.out.println("initialEncoderPosition: " + initialEncoderPosition);
      // System.out.println("currentEncoderPosition: " + m_intakeSubsystem.getEncoderPosition());
      return true;
    }
    return false;
  }
}
