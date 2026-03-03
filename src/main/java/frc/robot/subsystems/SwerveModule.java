package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModule {
    private SparkFlex SteerMotor, DriveMotor;
    private SparkFlexConfig SteerConfig, DriveConfig;
    private RelativeEncoder SteerEncoder;
    private SparkAnalogSensor SteerAnalogSensor;
    private RelativeEncoder DriveEncoder;
    private SparkClosedLoopController DrivePID, SteerPID;
    private SwerveModuleState DesiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double SteerOffset = 0;

    public SwerveModule(int driveMotorID, int steerMotorID, double steerOffset) {
        // Motors
        SteerMotor = new SparkFlex(steerMotorID, MotorType.kBrushless);
        DriveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);

        // encoders
        SteerEncoder = SteerMotor.getEncoder();
        SteerAnalogSensor = SteerMotor.getAnalog();
        DriveEncoder = DriveMotor.getEncoder();

        // PIDs
        SteerPID = SteerMotor.getClosedLoopController();
        DrivePID = DriveMotor.getClosedLoopController();

        // Motor Configuration
        SteerConfig = new SparkFlexConfig();
        SteerConfig.inverted(true);
        SteerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(Constants.SteeringPgain)
            .i(Constants.SteeringIgain)
            .d(Constants.SteeringDgain)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .outputRange(-1, 1);
        SteerConfig.encoder
            .positionConversionFactor(Constants.SteerMotorPositionFactor);
        SteerMotor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.SteerOffset = steerOffset;

        DriveConfig = new SparkFlexConfig();
        DriveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        DriveConfig.encoder
            .positionConversionFactor(Constants.DriveMotorPositionFactor)
            .velocityConversionFactor(Constants.DriveMotorVelocityFactor);
        DriveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.DrivingPgain)
            .i(Constants.DrivingIgain)
            .d(Constants.DrivingDgain)
            .outputRange(-1, 1)
            .feedForward.kV(Constants.DrivingFFgain);
        DriveMotor.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                DriveEncoder.getVelocity(),
                new Rotation2d(getWrappedPosition()));
    }

    public SwerveModuleState getDesiredPosition() {
        return DesiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                DriveEncoder.getPosition(),
                new Rotation2d(getWrappedPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize desired state based on current angle (never rotate module more than
        // 90degrees)
        desiredState.optimize(new Rotation2d(getWrappedPosition()));

        // Always set the driving motor's speed
        DrivePID.setSetpoint(desiredState.speedMetersPerSecond, ControlType.kVelocity);

        // But only set the steering motor's position if the driving motor is moving
        if (Math.abs(desiredState.speedMetersPerSecond) > Constants.kMinSpeedMetersPerSecond) {
            SteerPID.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);
        }

        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void setDesiredStateNoRestrictions(SwerveModuleState desiredState) {
        // Optimize desired state based on current angle (never rotate module more than
        // 90degrees)
        desiredState.optimize(new Rotation2d(getWrappedPosition()));

        // Always set the driving motor's speed
        DrivePID.setSetpoint(desiredState.speedMetersPerSecond, ControlType.kVelocity);

        // Always set the steering motor's position
        SteerPID.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);

        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void stop() {
        DriveMotor.set(0.0);
        SteerMotor.set(0.0);
    }

    public void resetEncoders() {
        DriveEncoder.setPosition(0);
        SteerEncoder.setPosition(getVirtualSteerPosition());
    }

    public double getVirtualSteerPosition(){
        double voltage = SteerAnalogSensor.getVoltage();
        double busVoltage = RobotController.getVoltage5V();

        double analogPosition = (voltage / busVoltage) * (Math.PI * 2);

        return analogPosition - SteerOffset;
    }

    public double getWrappedPosition(){
        double angle = SteerEncoder.getPosition();
        return angle - (2 * Math.PI) * Math.floor(angle / (2 * Math.PI));
    }
}
