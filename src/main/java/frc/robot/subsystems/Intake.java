// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {
  private SparkMax ExtensionMotor = new SparkMax(Constants.IntakeExtensionMotorID, MotorType.kBrushless);
  private SparkMax SpinningMotor = new SparkMax(Constants. IntakeSpinningMotorID, MotorType.kBrushless);
  private SparkMaxConfig ExtensionConfig = new SparkMaxConfig();
  private SparkMaxConfig SpinningConfig = new SparkMaxConfig();
  private SparkClosedLoopController ExtensionPID = ExtensionMotor.getClosedLoopController();
  private SparkClosedLoopController SpinningPID = SpinningMotor.getClosedLoopController();

  /** Creates a new Intake. */
  public Intake() {
    ExtensionConfig.inverted(false);
    ExtensionConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(Constants.IntakePgain, Constants.IntakeIgain, Constants.IntakeDgain)
      .positionWrappingEnabled(false)
      .outputRange(-1*Constants.IntakeExtensionMaxSpeed, Constants.IntakeExtensionMaxSpeed);
    ExtensionConfig.absoluteEncoder
      .positionConversionFactor(Constants.IntakePositionConversionFactor)
      .zeroOffset(Constants.IntakePositionOffset);
    ExtensionConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(Constants.IntakeForwardSoftLimit)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(Constants.IntakeReverseSoftLimit);
    ExtensionMotor.configure(ExtensionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SpinningConfig.inverted(false);
    SpinningConfig.closedLoop
      .pid(Constants.IntakeSpinPgain, Constants.IntakeSpinIgain, Constants.IntakeSpinDgain)
      .positionWrappingEnabled(false)
      .outputRange(0.0, 1.0);
    SpinningConfig.encoder
      .velocityConversionFactor(Constants.IntakeSpinVelocityConversionFactor);
    SpinningMotor.configure(SpinningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void SpinPickUp(){
    SpinningMotor.set(0.5);
  }
  public void SpinOut(){
    SpinningMotor.set(-0.5);
  }
  public void SpinAtSpeed(double speed){
    SpinningPID.setSetpoint(speed, ControlType.kVelocity);
  }
  @Log
  public double getSpinSpeed(){
    return SpinningMotor.getEncoder().getVelocity();
  }
  public void SpinStop(){
    SpinningMotor.set(0);
  }
  public void ExtensionOut(){
    ExtensionMotor.set(0.10);
  }
  public void ExtensionIn(){
    ExtensionMotor.set(-0.10);
  }
  public void ExtensionStop(){
    ExtensionMotor.set(0);
  }
  public void ExtensionToPosition(double position){
    ExtensionPID.setSetpoint(position, ControlType.kPosition);
  }
  @Log
  public boolean IsStowed() {
    return ExtensionMotor.getReverseSoftLimit().isReached();
  }
  @Log
  public boolean IsExtended(){
    return ExtensionMotor.getForwardSoftLimit().isReached();
  }
  @Log
  public double getExtensionPosition(){
    return ExtensionMotor.getAbsoluteEncoder().getPosition();
  }

  public Command spinPickupCommand(){
    return this.startEnd(this::SpinPickUp, this::SpinStop);
  }
  public Command spinStopCommand(){
    return this.runOnce(this::SpinStop);
  }
  public Command extendCommand(){
    return this.runOnce(()->{ExtensionToPosition(Constants.IntakeExtendedPosition);});
  }
  public Command stowCommand(){
    return this.runOnce(()->{ExtensionToPosition(Constants.IntakeStowedPosition);});
  }
  public Command ExtendOutCommand(){
    return this.startEnd(this::ExtensionOut, this::ExtensionStop);
  }
  public Command ExtendInCommand(){
    return this.startEnd(this::ExtensionIn, this::ExtensionStop);
  }
}
