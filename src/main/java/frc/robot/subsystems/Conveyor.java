// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private SparkFlex FloorMotor = new SparkFlex(Constants.FloorMotorID,MotorType.kBrushless);
  private SparkFlex KickerMotor = new SparkFlex(Constants.KickerMotorID, MotorType.kBrushless);
  private SparkFlexConfig FloorConfig = new SparkFlexConfig();
  private SparkFlexConfig KickerConfig = new SparkFlexConfig();

  /** Creates a new Converyor. */
  public Conveyor() {
    FloorConfig.inverted(true);
    FloorMotor.configure(FloorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    KickerConfig.inverted(false);
    KickerMotor.configure(KickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void FloorForward(){
    FloorMotor.set(0.80);
  }
  public void FloorBackward(){
    FloorMotor.set(-0.5);
  }
  public void FloorStop(){
    FloorMotor.set(0);
  }
  public void KickerForward(){
    KickerMotor.set(0.75);
  }
  public void KickerBackward(){
    KickerMotor.set(-0.75);
  }
  public void KickerStop(){
    KickerMotor.set(0);
  }

  public Command ConveyorMoveCommand(DoubleSupplier flywheelSpeedSupplier){
    return this.runEnd(
      ()->{
        double flywheelSpeed = flywheelSpeedSupplier.getAsDouble();

        if (flywheelSpeed >= Constants.FlywheelMinShotSpeed){
          // run conveyor to flywheel if flywheel is fast enough
          FloorForward();
          KickerForward();
        } else {
          // if flywheel is too slow, stop conveyor
          FloorStop();
        KickerStop();
        }
      },
      ()->{
        // end runnable, stop conveyor at end of command
        FloorStop();
        KickerStop();
      }
    );
  }
}
