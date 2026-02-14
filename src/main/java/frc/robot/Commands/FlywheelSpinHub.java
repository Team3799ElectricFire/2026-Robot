// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Hood;
import frc.robot.Subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlywheelSpinHub extends Command {
  private Shooter flywheel;
  private Hood hood;
  private DoubleSupplier hubSupplier; 
  
  /** Creates a new FlywheelSpinHub. */
  public FlywheelSpinHub(Shooter flywheel, Hood hood, DoubleSupplier hubSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheel = flywheel;
    this.hood = hood; 
    this.hubSupplier = hubSupplier;
    addRequirements(flywheel,hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double HubDistance = hubSupplier.getAsDouble();

    double Speed = CalcFlywheelSpeedFromDistance(HubDistance);
    double Position = CalcHoodPositionFromDistance(HubDistance);

    flywheel.FlywheelToSpeed(Speed);
    hood.setPosition(Position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.FlywheelStop();
    hood.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double CalcFlywheelSpeedFromDistance(double Distance){
    return 0; // TODO fill in function
  }
  private double CalcHoodPositionFromDistance(double Distance){
    return 0; //TODO fill in function
  } 
}
