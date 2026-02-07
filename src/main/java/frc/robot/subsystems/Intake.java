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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax ExtensionMotor = new SparkMax(Constants.IntakeExtensionMotorID, MotorType.kBrushless);
  private SparkMax SpinningMotor = new SparkMax(Constants. IntakeSpinningMotorID, MotorType.kBrushless);
  private SparkMaxConfig ExtensionConfig = new SparkMaxConfig();
  private SparkMaxConfig SpinningConfig = new SparkMaxConfig();
  private SparkClosedLoopController ExtensionPID = ExtensionMotor.getClosedLoopController();

  /** Creates a new Intake. */
  public Intake() {
    ExtensionConfig.inverted(false);
    ExtensionConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(Constants.IntakePgain, Constants.IntakeIgain, Constants.IntakeDgain)
      .positionWrappingEnabled(false)
      .outputRange(-1.0, 1.0);
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
  public void SpinStop(){
    SpinningMotor.set(0);
  }
  public void ExtensionOut(){
    ExtensionMotor.set(0.5);
  }
  public void ExtensionIn(){
    ExtensionMotor.set(-0.5);
  }
  public void ExtensionStop(){
    ExtensionMotor.set(0);
  }
  public void ExtensionToPosition(double position){
    ExtensionPID.setSetpoint(position, ControlType.kPosition);
  }
  public boolean IsStowed() {
    return ExtensionMotor.getReverseSoftLimit().isReached();
  }
  public boolean IsExtended(){
    return ExtensionMotor.getForwardSoftLimit().isReached();
  }
}
