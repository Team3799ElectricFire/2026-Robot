// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveShooting extends Command {
  private final Drivetrain drivetrain;
  private DoubleSupplier XSupplier, YSupplier;
  private SlewRateLimiter XLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter YLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private PIDController rotPID = new PIDController(Constants.teleopTurningPgain, 0, Constants.teleopTurningDgain);

  /** Creates a new DriveShooting. */
  public DriveShooting(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.XSupplier = xSupplier;
    this.YSupplier = ySupplier;

    rotPID.enableContinuousInput(-1*Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double XRawDemand = -1 * XSupplier.getAsDouble();
    double YRawDemand = -1 * YSupplier.getAsDouble();

    double leftMagnatude = Math.sqrt(XRawDemand * XRawDemand + YRawDemand * YRawDemand);

    boolean isDriving = leftMagnatude > Constants.minThumbstickMagnitude;

    if (!isDriving) {
      XRawDemand = 0;
      YRawDemand = 0;
    }

    double xDemand = XLimiter.calculate(XRawDemand * Math.abs(XRawDemand));
    double yDemand = YLimiter.calculate(YRawDemand * Math.abs(YRawDemand));
    double rotDemand = rotPID.calculate(
      MathUtil.angleModulus(drivetrain.getPose().getRotation().getRadians()),
      MathUtil.angleModulus(drivetrain.getHubAngle().getRadians()));

    // Only command the modules to move if the driver input is far enough from
    // center
    if (isDriving) {
      // Drive
      drivetrain.driveFieldRelative(xDemand, yDemand, rotDemand);
    } else {
      // Stop
      drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
