// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkFlex ClimbMotor  = new SparkFlex(Constants.ClimbMotorID, MotorType.kBrushless);
  private SparkClosedLoopController ClimbPID = ClimbMotor.getClosedLoopController();
  private SparkFlexConfig ClimbConfig = new SparkFlexConfig();
  //TODO Home Switch

  /** Creates a new Climber. */
  public Climber() {
    ClimbConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    ClimbConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0, 0, 0)
      .outputRange(0.0, 0.0); //TODO put actual numbers
    
    ClimbMotor.configure(ClimbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ClimbUp() { 
    ClimbMotor.set(0.5);
  } 
  public void ClimbDown () {
    ClimbMotor.set(-0.5);
  }
  public void ClimbStop() {
    ClimbMotor.set(0);
  }
  public void ClimbToPosition(double Position){
    ClimbPID.setSetpoint(Position, ControlType.kPosition);
  }

}
