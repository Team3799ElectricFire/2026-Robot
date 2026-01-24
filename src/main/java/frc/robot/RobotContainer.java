// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
  private XboxController driver = new XboxController(0);
  private XboxController codriver = new XboxController(1);
  private Drivetrain drivetrain = new Drivetrain();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // TODO Named commands for pathplanner 

    configureBindings();
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setAlliance(Alliance color) {
    //Cams.setAlliance(color);
    drivetrain.setAlliance(color);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
  }
}
