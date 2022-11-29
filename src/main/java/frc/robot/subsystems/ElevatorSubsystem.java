// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private WPI_TalonSRX elevatorMotor;
  public DigitalInput bottomLimitSwitch;
  public DigitalInput middleLimitSwitch;
  public DigitalInput topLimitSwitch;

  public ElevatorSubsystem() { 
    // Initializes a DigitalInput on DIO 0
    bottomLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_SWITCH);
    middleLimitSwitch = new DigitalInput(Constants.MIDDLE_LIMIT_SWITCH);
    topLimitSwitch = new DigitalInput(Constants.TOP_LIMIT_SWITCH);

    elevatorMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR_CHANNEL);
    elevatorMotor.configFactoryDefault();
    /* Config the sensor used for Primary PID and sensor direction */
    // intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
    //                                         Constants.kPIDLoopIdx,
    //                                         Constants.kTimeoutMs);
    /* Ensure sensor is positive when output is positive */
    elevatorMotor.setSensorPhase(Constants.kSensorPhase);
    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase. 
     */ 
    elevatorMotor.setInverted(Constants.kMotorInvert);
    // intakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //make isAtBottom return based off if the bottom limit switch is on or off
  public boolean isAtBottom(){
    if (bottomLimitSwitch.get()){//if bottom switch is OPEN
      return true;
    } //if bottom switch is CLOSED
      return false;
  }

  //make isAtMid return based off if the mid limit switch is on or off
  public boolean isAtMid(){
    if (middleLimitSwitch.get()){//if middle swithc is OPEN
      return true;
    } //if middle switch is CLOSED
    return false; 
  }

  public boolean isAtTop(){
    //make isAtTop return based off if the top limit switch is on or off
    if (topLimitSwitch.get()){//if top limit switch is OPEN
      return true;
    } //if top switch is CLOSED
    return false;
  }

  public void setElevatorSpeed(double speed){
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }
}
