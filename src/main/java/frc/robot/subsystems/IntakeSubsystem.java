// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonSRX intakeMotor;
  
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR_CHANNEL);
    intakeMotor.configFactoryDefault();
    /* Config the sensor used for Primary PID and sensor direction */
    // intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
    //                                         Constants.kPIDLoopIdx,
    //                                         Constants.kTimeoutMs);
    /* Ensure sensor is positive when output is positive */
		intakeMotor.setSensorPhase(Constants.kSensorPhase);
    /**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		intakeMotor.setInverted(Constants.kMotorInvert);
		// intakeMotor.setInverted(true);

		/* Config the peak and nominal outputs, 12V means full */
		// intakeMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		// intakeMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		// intakeMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		// intakeMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void spinIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
    System.out.println(getEncoderPosition());
  }


  public double getEncoderPosition() {
    return intakeMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx);
  }
}
