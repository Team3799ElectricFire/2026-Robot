// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveShooting_AUTO extends Command {
  private final Drivetrain drivetrain;
  private PIDController rotPID = new PIDController(Constants.teleopTurningPgain, 0, Constants.teleopTurningDgain);

  /** Creates a new DriveShooting. */
  public DriveShooting_AUTO(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    rotPID.enableContinuousInput(-1*Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotDemand = rotPID.calculate(
      MathUtil.angleModulus(drivetrain.getPose().getRotation().getRadians()),
      MathUtil.angleModulus(drivetrain.getHubAngle().getRadians())
    );

    // Drive
    drivetrain.driveFieldRelative(0.0, 0.0, rotDemand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getFacingHub();
  }
}
