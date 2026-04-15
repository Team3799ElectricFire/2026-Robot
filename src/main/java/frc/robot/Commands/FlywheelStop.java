// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Hood;
import frc.robot.Subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlywheelStop extends Command {
  private Shooter flywheel;
  private Hood hood;

  /** Creates a new FlywheelIdle. */
  public FlywheelStop(Shooter flywheel, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheel = flywheel;
    this.hood = hood;
    addRequirements(flywheel,hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.FlywheelStop();
    hood.setPosition(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // end immediately
  }
}
