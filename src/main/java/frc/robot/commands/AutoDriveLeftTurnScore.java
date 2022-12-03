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
public class AutoDriveLeftTurnScore extends SequentialCommandGroup {
  /** Creates a new AutoDriveLeftTurnScore. */
  public AutoDriveLeftTurnScore(DriveTrainSubsystem driveTrainSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new DriveTrainCommand(driveTrainSubsystem, Constants.AUTO_SPEED_1, Constants.AUTO_SPEED_1).withTimeout(Constants.TURN_TIMEOUT),
        new ElevatorUpCommand(elevatorSubsystem, true)
      ),
      new AutoTurn(driveTrainSubsystem, Constants.TURNING_SPEED, false),//need to test turning speed
      new DriveTrainCommand(driveTrainSubsystem, Constants.AUTO_SPEED_2, Constants.AUTO_SPEED_2),
      new ElevatorUpCommand(elevatorSubsystem, false)
    );
  }
}
