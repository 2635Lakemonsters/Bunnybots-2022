// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends CommandBase {
  /** Creates a new ElevatorDownCommand. */
  public boolean atBottom;
  private ElevatorSubsystem m_elevatorSubsystem;

  public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("down command button pressed: is going to mid TODO");
    // System.out.println("Bottom: "+m_elevatorSubsystem.isAtBottom());
    // System.out.println("Mid: "+m_elevatorSubsystem.isAtMid());
    // System.out.println("Top: "+m_elevatorSubsystem.isAtTop());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atBottom = m_elevatorSubsystem.isAtBottom();
    m_elevatorSubsystem.setElevatorSpeed(Constants.ELEVATOR_DOWN_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (atBottom){
      return true;
    } else if (!atBottom){
      return false;
    }
    return false;
  }
}
