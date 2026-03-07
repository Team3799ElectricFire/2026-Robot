package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    // PID swerve constants
    public static final double SteeringPgain = 2.5;
    public static final double SteeringIgain = 0.001;
    public static final double SteeringDgain = 3; // Max derivative gain is 3 (according to REV hardware client)
    public static final double DrivingPgain = 0.1;
    public static final double DrivingIgain = 0;
    public static final double DrivingDgain = 0;
    public static final double DrivingFFgain = 1.0/565.0;

    // intake pid
    public static final double IntakePgain = 0.007;
    public static final double IntakeIgain = 0;
    public static final double IntakeDgain = 0;
    public static final double IntakePositionConversionFactor = 360.0;
    public static final double IntakePositionOffset = 187.8/360.0;
    public static final double IntakeForwardSoftLimit = 160.0;
    public static final double IntakeReverseSoftLimit = 88.0;
    public static final double IntakeExtendedPosition = 158.8;
    public static final double IntakeStowedPosition = 89.0;
    public static final double IntakeExtensionMaxSpeed = 0.25;
    public static final double IntakeSpinPgain = 0; // TODO tune for velocity mode and change command back
    public static final double IntakeSpinIgain = 0;
    public static final double IntakeSpinDgain = 0;
    public static final double IntakeSpinVelocityConversionFactor = (1.0/3.0) * (15.0/20.0);
    public static final double IntakeSpeed = 1700.0;

    // climb pid
    public static final double ClimberPgain = 1.25;
    public static final double ClimberIgain = 0;
    public static final double ClimberDgain = 0.001;
    public static final double ClimberPositionConversionFactor = (1.0/44.1);
    public static final double ClimbUpPosition = 3.85;
    public static final double ClimbDownPosition = 0.0;

    // pew pew pid
    public static final double ShooterPgain = 0.0007;
    public static final double ShooterIgain = 0.0;
    public static final double ShooterDgain = 0.0;
    public static final double ShooterKs = 0.24;
    public static final double ShooterKv = 0.0021054;
    public static final double FlywheelPassSpeed = 2700.0;
    public static final double HoodPassPosition = 0.7;

    // Conversion factors
    public static final double DriveMotorPositionFactor = 0.0521375063; // meters
    public static final double DriveMotorVelocityFactor = DriveMotorPositionFactor/60.0; // meters per sec
    public static final double SteerMotorPositionFactor = (2 * Math.PI) / (150.0 / 7.0); // radians per rotation

    // swerves limits
    public static final double kMinSpeedMetersPerSecond = 0.1;
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(19.3);
    public static final double kMaxAngularSpeed = Units.degreesToRadians(360);
    public static final double kFrontRightChassisAngularOffset = Units.degreesToRadians(180.33); // CAN ID 7
    public static final double kFrontLeftChassisAngularOffset = Units.degreesToRadians(98.98); // CAN ID 1
    public static final double kBackRightChassisAngularOffset = Units.degreesToRadians(274.12); // CAN ID 5
    public static final double kBackLeftChassisAngularOffset = Units.degreesToRadians(1.71);  // CAN ID 3
    
    // driving 
    public static final double panRateOfChangeLimit = 10.0;
    public static final double rotRateOfChangeLimit = 10.0;
    public static final double minThumbstickMagnitude = 0.1;
    public static final double teleopTurningPgain = 1.5;
    public static final double teleopTurningDgain = 0.01;

    // CanbusID
    public static final int FrontRightDriveMotorID = 8;
    public static final int FrontRightSteerMotorID = 7;
    public static final int FrontLeftDriveMotorID = 2;
    public static final int FrontLeftSteerMotorID = 1;
    public static final int BackRightDriveMotorID = 6;
    public static final int BackRightSteerMotorID = 5;
    public static final int BackLeftDriveMotorID = 4;
    public static final int BackLeftSteerMotorID = 3;
    public static final int PidgeonID = 18;
    public static final int ClimbMotorID = 20; 
    public static final int IntakeExtensionMotorID = 16;
    public static final int IntakeSpinningMotorID = 17;
    public static final int LeftFlywheelMotorID = 21;
    public static final int RightFlywheelMotorID = 22;
    public static final int FloorMotorID = 10;
    public static final int KickerMotorID = 11;
    
    // pwm channel
    public static final int LeftHoodServoID = 0;
    public static final int RightHoodServoID = 1;

     // Kinematics
    public static final double WheelBase = Units.inchesToMeters(21.75);
    public static final Translation2d FrontRightTranslation = new Translation2d(+WheelBase * 0.5, -WheelBase * 0.5);
    public static final Translation2d FrontLeftTranslation = new Translation2d(+WheelBase * 0.5, +WheelBase * 0.5);
    public static final Translation2d BackRightTranslation = new Translation2d(-WheelBase * 0.5, -WheelBase * 0.5);
    public static final Translation2d BackLeftTranslation = new Translation2d(-WheelBase * 0.5, +WheelBase * 0.5);
    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        FrontRightTranslation,
        FrontLeftTranslation,
        BackRightTranslation,
        BackLeftTranslation);

    // Path Planner
    public static final double MassKG = 57;
    public static final double MOI = 6.883;
    public static final ModuleConfig SwerveConfig = new ModuleConfig(
        Units.inchesToMeters(2),
        kMaxSpeedMetersPerSecond,
        1.0, 
        DCMotor.getNeoVortex(1).withReduction(6.12),
        50,
        1);
    public static final RobotConfig ROBOTCONFIG = new RobotConfig(
        MassKG,
        MOI,
        SwerveConfig,
        FrontRightTranslation, FrontLeftTranslation, BackRightTranslation, BackLeftTranslation);
    public static final PIDConstants TranslationPIDconstants = new PIDConstants(
        30,
        0.75,
        0.0);
    public static final PIDConstants RotationPIDconstants = new PIDConstants(
        13.0,
        0.20,
        0.0);
    
    public static final double HighSpeedMetersPerSecond = kMaxSpeedMetersPerSecond;
    public static final double LowSpeedMetersPerSecond = Units.feetToMeters(10.0);
    
    public static final Translation2d kBlueHub = new Translation2d(
        Units.inchesToMeters(182.1),
        Units.inchesToMeters(158.85));
    public static final Translation2d kRedHub = new Translation2d(
        Units.inchesToMeters(469.1),
        Units.inchesToMeters(158.85));
    public static final double kFacingHubTolerance = 10.0;
}
