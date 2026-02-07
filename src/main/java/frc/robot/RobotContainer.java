// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Cameras.VisionSample;

public class RobotContainer {
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController codriver = new CommandXboxController(1);
  private Drivetrain drivetrain = new Drivetrain();
  private Cameras cameras = new Cameras();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // TODO Named commands for pathplanner 
    // 

    configureBindings();
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setAlliance(Alliance color) {
    //Cams.setAlliance(color);
    drivetrain.setAlliance(color);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new DriveDefault(drivetrain,driver::getLeftY, driver::getLeftX, driver::getRightX));
    driver.rightBumper().whileTrue(new DriveIntake(drivetrain,driver::getLeftY,driver::getLeftX));
    driver.rightTrigger().whileTrue(new DriveShooting(drivetrain,driver::getLeftY,driver::getLeftX));
  }

  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
  }

   public void correctOdometry() {
    List<VisionSample> visionSamples = cameras.flushSamples();
    cameras.updateSpeeds(drivetrain.getRobotRelativeSpeeds());

    for (var sample : visionSamples) {
      double thetaStdDev = sample.weight() > 0.9 ? 10.0 : 99999.0;
      drivetrain.addVisionMeasurement(
        sample.pose(), 
        sample.timestamp(), 
        VecBuilder.fill(0.1 / sample.weight(), 0.1 / sample.weight(), thetaStdDev)
      );
    }
  }
}
