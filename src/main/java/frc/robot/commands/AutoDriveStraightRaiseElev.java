// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveStraightRaiseElev extends SequentialCommandGroup {
  /** Creates a new AutonomousGroup. */
  public AutoDriveStraightRaiseElev(DriveTrainSubsystem driveTrainSubsystem, ElevatorSubsystem elevatorSubsystem) {

    addCommands(
      //drive fast and spin freely, end after 3 sec
      new ParallelCommandGroup(
        //drive train
        // new SequentialCommandGroup (
          //front drives 251.5" fast then 10" slow
          //distance to table from edge of field = 300"
          //length of robot w/ bumpers : 38.5"
          //slow distance from front desired: 10"
          //fast distance from front : 300 - robot length - slow distance" = 251.5"
          new NavxDriveCommand(driveTrainSubsystem, Constants.AUTO_SPEED_1).withTimeout(Constants.STRAIGHT_TIMEOUT) //),
         
        //store elev
        , new ElevatorUpCommand(elevatorSubsystem, Constants.ELEV_HALF_UP)
      ), 
      new NavxDriveCommand(driveTrainSubsystem, Constants.AUTO_SPEED_2),

      //score
      new ElevatorUpCommand(elevatorSubsystem, Constants.ELEV_FULL_UP)
    );

  }

}
