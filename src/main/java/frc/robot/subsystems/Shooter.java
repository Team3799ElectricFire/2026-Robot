// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private SparkFlex LeftFlywheelMotor = new SparkFlex(Constants.LeftFlywheelMotorID, MotorType.kBrushless);
  private SparkFlex RightFlywheelMotor = new SparkFlex(Constants.RightFlywheelMotorID, MotorType.kBrushless);
  private SparkFlexConfig RightFlywheelConfig = new SparkFlexConfig();
  private SparkFlexConfig LeftFlywheelConfig = new SparkFlexConfig();
  private SparkClosedLoopController FlywheelPID = LeftFlywheelMotor.getClosedLoopController();

  /** Creates a new Shooter. */
  public Shooter() {
    RightFlywheelConfig.follow(LeftFlywheelMotor,true);
    RightFlywheelMotor.configure(RightFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftFlywheelConfig.inverted(false);
    LeftFlywheelConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0, 0, 0); //TODO tune pid
    LeftFlywheelConfig.encoder.velocityConversionFactor(1.0);//TODO get conversion from mechanical ppl
    LeftFlywheelMotor.configure(LeftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void FlywheelForward(){
    LeftFlywheelMotor.set(0.5);
  }
  public void FlywheelBackward(){
   LeftFlywheelMotor.set(-0.5); 
  }
  public void FlywheelStop(){
    LeftFlywheelMotor.set(0);
  }
  public void FlywheelToSpeed(double Speed){
    FlywheelPID.setSetpoint(Speed, ControlType.kVelocity);
  }
  public boolean IsSpeed(){
    return FlywheelPID.isAtSetpoint();
  }
}
