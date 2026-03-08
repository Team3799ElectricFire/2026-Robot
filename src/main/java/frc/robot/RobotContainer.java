// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
// import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Cameras;
import frc.robot.Subsystems.Cameras.VisionSample;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Hood;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

@Logged
public class RobotContainer {
  private CommandXboxController driver = new CommandXboxController(0);
  // private CommandXboxController codriver = new CommandXboxController(1);
  private Drivetrain drivetrain = new Drivetrain();
  private Cameras cameras = new Cameras(Cameras.camerasFromConfigs(VisionConstants.CONFIGS));
  private Climber climber = new Climber();
  private Conveyor conveyor = new Conveyor();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Hood hood = new Hood();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Named commands for pathplanner 
    NamedCommands.registerCommand("Climber Up", climber.CimberToPositionCommand(Constants.ClimbUpPosition));
    NamedCommands.registerCommand("Cliber Down", climber.CimberToPositionCommand(Constants.ClimbDownPosition));
    NamedCommands.registerCommand("Shoot", conveyor.ConveyorMoveCommand().withTimeout(5.0));
    NamedCommands.registerCommand("Run Intake", new IntakePickUp(intake).withTimeout(10.0));
    NamedCommands.registerCommand("Stop Intake", intake.spinStopCommand());
    NamedCommands.registerCommand("Run Flywheel", new FlywheelSpinHub(shooter, hood, drivetrain::getHubDistance).until(shooter::IsSpeed));
    NamedCommands.registerCommand("Turn To Hub", new DriveShooting(drivetrain, ()->{return 0;}, ()->{return 0;}));

    configureBindings();
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setAlliance(Alliance color) {
    drivetrain.setAlliance(color);
  }

  Command intakingCommand = new ParallelCommandGroup(
    // new DriveIntake(drivetrain,driver::getLeftY,driver::getLeftX),
    new IntakePickUp(intake)
  );

  Command hubShootCommand = new ParallelCommandGroup(
    new DriveShooting(drivetrain,driver::getLeftY,driver::getLeftX),
    new FlywheelSpinHub(shooter, hood, drivetrain::getHubDistance)
    // shooter.FlywheelToSpeedCommand(2600)
  );

  Command passShootCommand = new FlywheelSpinPass(shooter, hood);

  private void configureBindings() {
    drivetrain.setDefaultCommand(new DriveDefault(drivetrain,driver::getLeftY, driver::getLeftX, driver::getRightX));
    
    driver.rightBumper().whileTrue(intakingCommand);
    driver.rightTrigger().whileTrue(hubShootCommand);
    driver.leftTrigger().whileTrue(passShootCommand);
    driver.a().whileTrue(conveyor.ConveyorMoveCommand());
    driver.b().onTrue(intake.stowCommand());
    driver.povUp().whileTrue(climber.ClimberUpCommand());
    driver.povDown().whileTrue(climber.CliberDownCommand());
    
    // codriver.a().whileTrue(conveyor.ConveyorMoveCommand());
    // codriver.leftTrigger().whileTrue(passShootCommand);
    // codriver.rightTrigger().whileTrue(hubShootCommand);
    // codriver.rightBumper().whileTrue(intakingCommand);
    // codriver.povUp().whileTrue(climber.ClimberUpCommand());
    // codriver.povDown().whileTrue(climber.CliberDownCommand());

    SmartDashboard.putData("Spin Intake", intake.spinPickupCommand());
    SmartDashboard.putData("Extend Intake" , intake.extendCommand());
    SmartDashboard.putData("Stow Intake" , intake.stowCommand());
    SmartDashboard.putData("Test Flywheel" , new FlywheelTest(shooter, hood));
    SmartDashboard.putData("Extend Climber", climber.CimberToPositionCommand(Constants.ClimbUpPosition));
    SmartDashboard.putData("Retract Climber", climber.CimberToPositionCommand(Constants.ClimbDownPosition));
    SmartDashboard.putData("Zero Heading", drivetrain.ZeroHeadingCommand());
    SmartDashboard.putData("Reset to Starting Position", drivetrain.resetPoseCommand(Constants.StartingPose));
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

  public void resetEncoders(){
    drivetrain.resetEncoders();
  }
}
