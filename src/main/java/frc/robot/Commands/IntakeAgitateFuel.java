// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAgitateFuel extends Command {
  private final Intake intake;
  private Timer posTimer = new Timer();
  private double maxPos = Constants.IntakeExtendedPosition;
  private double minPos = Constants.IntakeStowedPosition;
  private double meanPos = (minPos + maxPos) / 2.0;
  private double amplitude = (maxPos - minPos) / 2.0;
  private double period = 5.0; // seconds for full range of motion

  /** Creates a new IntakeAgitateFuel. */
  public IntakeAgitateFuel(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.SpinPickUp();
    posTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = posTimer.get();

    double targetPos = amplitude * Math.cos(2*Math.PI * (t / period)) + meanPos;

    intake.ExtensionToPosition(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.ExtensionToPosition(Constants.IntakeStowedPosition);
    intake.SpinStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
