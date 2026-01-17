package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    
    // Conversion factors
    public static final double DriveMotorPositionFactor = 0.0521375063; // meters
    public static final double DriveMotorVelocityFactor = DriveMotorPositionFactor/60.0; // meters per sec
    public static final double SteerMotorPositionFactor = 2 * Math.PI; // radians

    // swerves limits
    public static final double kMinSpeedMetersPerSecond = 0.1;
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(19.3);
    public static final double kMaxAngularSpeed = Units.degreesToRadians(360);
    public static final double kFrontRightChassisAngularOffset = 0.7922185; // CAN ID 7
    public static final double kFrontLeftChassisAngularOffset = 0.2272288; // CAN ID 1
    public static final double kBackRightChassisAngularOffset = 0.4184715; // CAN ID 5
    public static final double kBackLeftChassisAngularOffset = 0.0312085;  // CAN ID 3

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

     // Kinematics
    public static final double WheelBase = Units.inchesToMeters(23.75);
    public static final Translation2d FrontRightTranslation = new Translation2d(+WheelBase / 2, -WheelBase / 2);
    public static final Translation2d FrontLeftTranslation = new Translation2d(+WheelBase / 2, +WheelBase / 2);
    public static final Translation2d BackRightTranslation = new Translation2d(-WheelBase / 2, -WheelBase / 2);
    public static final Translation2d BackLeftTranslation = new Translation2d(-WheelBase / 2, +WheelBase / 2);
    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            FrontRightTranslation,
            FrontLeftTranslation,
            BackRightTranslation,
            BackLeftTranslation);

    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1,0.1,0.1);
    public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(1,1,1);

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
    

    
}
