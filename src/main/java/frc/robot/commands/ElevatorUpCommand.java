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

    //System.out.println("IT WORKS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(m_elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("up command button pressed: is going to mid TODO");
    // System.out.println("Bottom: "+m_elevatorSubsystem.isAtBottom());
    // System.out.println("Mid: "+m_elevatorSubsystem.isAtMid());
    // System.out.println("Top: "+m_elevatorSubsystem.isAtTop());
  }

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
    System.out.println("NavxDriveCommand ENDED!!!!!!!!!!!!!!!!!!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isGoingToMid){  //is it going to mid
      if (atMid || atTop){ //region 1 or if goes over
        System.out.println("mid switch hit");
        System.out.println("top2 switch hit");
        return true;
      } else{
        return false;
      }
    } else if (!isGoingToMid){
      if (atTop){
        System.out.println("top1 switch hit");
        return true;
      } else {
        return false;
      }
    }
    return true;
  }
}
