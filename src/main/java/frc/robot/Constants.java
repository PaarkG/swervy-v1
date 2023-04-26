package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveConstants {
        public static final double kWheelDiameter = 3.0;
        public static final double kTrackWidth = Units.inchesToMeters(28);
        public static final double kWheelBase = Units.inchesToMeters(28);
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kDirectionSlewRate = 1.2;
        public static final double kMagnitudeSlewRate = 1.8;
        public static final double kRotationalSlewRate = 2.0;
        

        // SINCE MODULES ARE ALL ROTATED 90 DEGREES FROM THE NEAREST ONES
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // CAN BUS IDs
        public static final int kFrontLeftDriveMotorID = 1;
        public static final int kFrontLeftTurningMotorID = 2;
        public static final int kFrontRightDriveMotorID = 3;
        public static final int kFrontRightTurningMotorID = 4;
        public static final int kBackLeftDriveMotorID = 5;
        public static final int kBackLeftTurningMotorID = 6;
        public static final int kBackRightDriveMotorID = 7;
        public static final int kBackRightTurningMotorID = 8;

        // MODULE INFO
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final boolean kTurningEncoderInverted = true;
        public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
        public static final double kTurningEncoderPositionPIDMinInput = 0;
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;

        // PIDF for MODULES
        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        // CURRENT LIMITS
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static class InputConstants {
        public static final int kDriveControllerPort = 0;
    }
}
