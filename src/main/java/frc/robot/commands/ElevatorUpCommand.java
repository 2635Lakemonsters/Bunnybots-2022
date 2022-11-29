// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends CommandBase {
  /** Creates a new ElevatorUpCommand. */
  private ElevatorSubsystem m_elevatorSubsystem;
  private boolean isGoingToMid;

  public boolean atMid;
  public boolean atTop;

  public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem, boolean isGoingToMid) {
    this.isGoingToMid = isGoingToMid; 

    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(m_elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setElevatorSpeed(Constants.ELEVATOR_UP_SPEED);
    atMid = m_elevatorSubsystem.isAtMid();
    atTop = m_elevatorSubsystem.isAtTop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isGoingToMid){
      if (atMid || atTop){
        return true;
      } else if (!atMid || !atTop){
        return false;
      }
    } else if (!isGoingToMid){
      if (atTop){
        return true;
      } else if (!atTop){
        return false;
      }
    }
    return true;
  }
}
