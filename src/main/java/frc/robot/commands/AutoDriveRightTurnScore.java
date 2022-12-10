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
public class AutoDriveRightTurnScore extends SequentialCommandGroup {
  /** Creates a new AutoDriveLeftTurnScore. */
  public AutoDriveRightTurnScore(DriveTrainSubsystem driveTrainSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        //Robot length edge-middle w/ bumpers : 19.25"
        //Distance from edge field - middle of tote @ side of scoring tower : 315"
        //want robot middle to go to 315" from edge of field prior to turn
        //robot middle goes 295.75"(total)
        //Timeout stops when middle has gone 295.75"
        new DriveTrainCommand(driveTrainSubsystem, Constants.AUTO_SPEED_1, Constants.AUTO_SPEED_1).withTimeout(Constants.TURN_TIMEOUT),
        new ElevatorUpCommand(elevatorSubsystem, Constants.ELEV_HALF_UP)
      ),
      new AutoTurn(driveTrainSubsystem, Constants.TURNING_SPEED, true),//need to test turning speed
      new DriveTrainCommand(driveTrainSubsystem, Constants.AUTO_SPEED_2, Constants.AUTO_SPEED_2),
      new ElevatorUpCommand(elevatorSubsystem, Constants.ELEV_FULL_UP)
    );
  }
}
