package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0/7.0);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 1.15;
        public static final double kITurning = 0.0;
        public static final double kDTurning = 0.0;
        public static final double kPDriving = 0.05;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(20.25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20.25);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(

                // was working
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kFrontRightDriveMotorPort =7;
        public static final int kBackRightDriveMotorPort = 1;

        public static final int kFrontLeftTurningMotorPort = 6;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 8;
        public static final int kBackRightTurningMotorPort = 2; 

        
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // Did you make sure to set BootToAbsolute??
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.87;// 0.857; //4.16; //4.49; // ID 3
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =  5.10;//2.204; //4.72; //5.39; // ID 0 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =  5.37;//5.118; //5.06; //5.05; // ID 2
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  4.47;//1.334; //0.48; //0.81; // ID 1

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.6;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond ;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5; //Was 10
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kShooterControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.3;
    }

    public static final class VisionConstants {
        public static final double cameraHeight = 21.25; //Inches
        public static final double apriltagHeight = 57.0; //Inches
        public static final double cameraAngle = 16; //Degrees
    }

    public static final class ShooterConstants {
        
        public static final double degreePerRot = 2.05;
        public static final double goalHeight = 83.0;//inches
        public static final int intake = 11;
        public static final int shooter1 = 9;
        public static final int shooter2 = 10;
}
    public static final class PivotConstants {
        public static final int pivotID = 12;
    }

    public static final class ClimberConstants {
        public static final int climberID1 = 13;
        public static final int climberID2 = 14;
    }

}