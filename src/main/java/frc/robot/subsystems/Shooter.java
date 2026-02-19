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
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged {
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
      .pid(Constants.ShooterPgain, Constants.ShooterIgain, Constants.ShooterDgain)
      .feedForward.kS(Constants.ShooterKs).kV(Constants.ShooterKv); 
    LeftFlywheelConfig.encoder.velocityConversionFactor(1.0);
    LeftFlywheelMotor.configure(LeftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void FlywheelForward(){
    LeftFlywheelMotor.set(0.5);
  }
  public void FlywheelVoltage(double Volts){
    LeftFlywheelMotor.setVoltage(Volts);
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
  @Log
  public boolean IsSpeed(){
    return FlywheelPID.isAtSetpoint();
  }
  @Log 
  public double getSpeed(){
    return LeftFlywheelMotor.getEncoder().getVelocity();
  }
  @Log 
  public double getTargetSpeed(){
    return FlywheelPID.getSetpoint();
  }
}
