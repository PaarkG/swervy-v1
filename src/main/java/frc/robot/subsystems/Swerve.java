package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.SwerveUtils;

public class Swerve {
    private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.kFrontLeftDriveMotorID, 
        SwerveConstants.kFrontLeftTurningMotorID, SwerveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule frontRight = new SwerveModule(SwerveConstants.kFrontRightDriveMotorID, 
        SwerveConstants.kFrontRightTurningMotorID, SwerveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule backLeft = new SwerveModule(SwerveConstants.kBackLeftDriveMotorID, 
        SwerveConstants.kBackLeftTurningMotorID, SwerveConstants.kBackLeftChassisAngularOffset);
        
    private final SwerveModule backRight = new SwerveModule(SwerveConstants.kBackRightDriveMotorID, 
        SwerveConstants.kBackRightTurningMotorID, SwerveConstants.kBackRightChassisAngularOffset);

    private final AHRS gyro = new AHRS();

    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;
    
    private SlewRateLimiter magLimiter = new SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(SwerveConstants.kRotationalSlewRate);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        SwerveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    private static Swerve instance;

    public static Swerve getInstance() {
        if(instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public void updateOdometry() {
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), 
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
        double xSpeedCommanded;
        double ySpeedCommanded;
    
        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
    
            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(SwerveConstants.kDirectionSlewRate / currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }
          
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
            if (angleDif < 0.45*Math.PI) {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            }
            else if (angleDif > 0.85*Math.PI) {
                if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                // keep currentTranslationDir unchanged
                currentTranslationMag = magLimiter.calculate(0.0);
            } else {
                currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            }
            } else {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(0.0);
            }
            prevTime = currentTime;
          
            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rot);
        } else {
          xSpeedCommanded = xSpeed;
          ySpeedCommanded = ySpeed;
          currentRotation = rot;
        }
    
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * SwerveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * SwerveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = currentRotation * SwerveConstants.kMaxAngularSpeed;
    
        var swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

    public double getTurnRate() {
        return gyro.getRate();
    }
}
