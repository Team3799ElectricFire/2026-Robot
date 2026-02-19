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
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Climber extends SubsystemBase implements Logged {
  private SparkFlex ClimbMotor  = new SparkFlex(Constants.ClimbMotorID, MotorType.kBrushless);
  private SparkClosedLoopController ClimbPID = ClimbMotor.getClosedLoopController();
  private SparkFlexConfig ClimbConfig = new SparkFlexConfig();

  /** Creates a new Climber. */
  public Climber() {
    ClimbConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    ClimbConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(Constants.ClimberPgain, Constants.ClimberIgain, Constants.ClimberDgain)
      .outputRange(-1.0, 1.0);
    ClimbConfig.encoder.positionConversionFactor(Constants.ClimberPositionConversionFactor);
    ClimbConfig.limitSwitch
      .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
      .reverseLimitSwitchType(Type.kNormallyOpen);
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
  @Log 
  public Boolean atPosition(){
    return ClimbPID.isAtSetpoint();
  }
  @Log
  public double getPosition(){
    return ClimbMotor.getEncoder().getPosition();
  }
  @Log 
  public double getSetpoint(){
    return ClimbPID.getSetpoint();
  }

  public Command ClimberUpCommand(){
    return this.startEnd(this::ClimbUp, this::ClimbStop);
  }
  public Command CliberDownCommand(){
    return this.startEnd(this::ClimbDown,this::ClimbStop);
  }
  public Command CimberToPositionCommand(double position){
     return this.startEnd(() -> {this.ClimbToPosition(position);}, this::ClimbStop).until(this::atPosition);
  }

}
