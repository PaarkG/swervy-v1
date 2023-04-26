package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkMaxPIDController drivePIDController;
    private final SparkMaxPIDController turningPIDController;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveID, int turningID, double chassisAngularOffset) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turningEncoder.setPositionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(SwerveConstants.kTurningEncoderVelocityFactor);
        turningEncoder.setInverted(SwerveConstants.kTurningEncoderInverted);

        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setFeedbackDevice(driveEncoder);
        drivePIDController.setP(SwerveConstants.kDrivingP);
        drivePIDController.setI(SwerveConstants.kDrivingI);
        drivePIDController.setD(SwerveConstants.kDrivingD);
        drivePIDController.setFF(SwerveConstants.kDrivingFF);
        drivePIDController.setOutputRange(SwerveConstants.kDrivingMinOutput, SwerveConstants.kDrivingMaxOutput);

        turningPIDController = turningMotor.getPIDController();
        turningPIDController.setFeedbackDevice(turningEncoder);
        turningPIDController.setP(SwerveConstants.kTurningP);
        turningPIDController.setI(SwerveConstants.kTurningI);
        turningPIDController.setD(SwerveConstants.kTurningD);
        turningPIDController.setFF(SwerveConstants.kTurningFF);
        turningPIDController.setOutputRange(SwerveConstants.kTurningMinOutput, SwerveConstants.kTurningMaxOutput);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(SwerveConstants.kDrivingMotorCurrentLimit);
        turningMotor.setSmartCurrentLimit(SwerveConstants.kTurningMotorCurrentLimit);

        driveMotor.burnFlash();
        turningMotor.burnFlash();

        this.chassisAngularOffset = chassisAngularOffset;
        this.desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = new SwerveModuleState();
        optimizedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        optimizedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
        optimizedDesiredState = SwerveModuleState.optimize(optimizedDesiredState, new Rotation2d(turningEncoder.getPosition()));

        drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = desiredState;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
}
