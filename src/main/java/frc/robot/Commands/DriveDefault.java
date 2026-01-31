// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDefault extends Command {
  private final Drivetrain drivetrain;
  private DoubleSupplier XSupplier, YSupplier, RotSupplier;
  private SlewRateLimiter XLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter YLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter RotLimiter = new SlewRateLimiter(Constants.rotRateOfChangeLimit);
  private Rotation2d rotationTarget = null;
  private PIDController rotPID = new PIDController(Constants.teleopTurningPgain, 0, Constants.teleopTurningDgain);
  
  /** Creates a new DriveDefault. */
  public DriveDefault(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
    this.drivetrain = drivetrain;
    this.XSupplier = xSupplier;
    this.YSupplier = ySupplier;
    this.RotSupplier = rotSupplier;

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
    double RotRawDemand = -1 * RotSupplier.getAsDouble();

    double leftMagnatude = Math.sqrt(XRawDemand * XRawDemand + YRawDemand * YRawDemand);
    double rightMagnatude = Math.abs(RotRawDemand);

    boolean isDriving = leftMagnatude > Constants.minThumbstickMagnitude;
    boolean isTurning = rightMagnatude > Constants.minThumbstickMagnitude;

    if (!isDriving) {
      XRawDemand = 0;
      YRawDemand = 0;
    }

     // If driver is not trying to turn, prevent rotation with PID
    if (isDriving && !isTurning) {
      // Just stopped turning, rotation target still not set
      if (rotationTarget == null) {
        // Set target to most recent heading
        rotationTarget = drivetrain.getPose().getRotation();
        rotPID.setSetpoint(MathUtil.angleModulus(rotationTarget.getRadians()));
      }

      // Calculate error between target and current heading
      RotRawDemand = rotPID.calculate(MathUtil.angleModulus(drivetrain.getPose().getRotation().getRadians()));
    } else {
      // Stopped driving OR started turning, stop trying to hold heading
      rotationTarget = null;
    }

    if (isTurning && !isDriving) {
      // sligtly reduce sensitivity if turning in place
      RotRawDemand = RotRawDemand * 0.9;
    }

    double xDemand = XLimiter.calculate(XRawDemand * Math.abs(XRawDemand));
    double yDemand = YLimiter.calculate(YRawDemand * Math.abs(YRawDemand));
    double rotDemand = RotLimiter.calculate(RotRawDemand * Math.abs(RotRawDemand));

    // Only command the modules to move if the driver input is far enough from
    // center
    if (isDriving || isTurning) {
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
