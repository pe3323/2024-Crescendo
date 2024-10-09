package frc.robot.subsystems;


import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getTable("Swerve")
        .getStructArrayTopic("current Array states", SwerveModuleState.struct).publish();

    StructArrayPublisher<SwerveModuleState> publisher2 = NetworkTableInstance.getDefault().getTable("Swerve")
    .getStructArrayTopic("Desired Array states", SwerveModuleState.struct).publish();

    //StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    //.getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);


    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem() {

        // SwerveModulePosition pos = new SwerveModulePosition();
        SwerveModulePosition []initpos = new SwerveModulePosition[] { new SwerveModulePosition(frontLeft.getDrivePosition(), getInitRotation2d(frontLeft)), new SwerveModulePosition(frontRight.getDrivePosition(), getInitRotation2d(frontRight)), 
            new SwerveModulePosition(backLeft.getDrivePosition(), getInitRotation2d(backLeft)), new SwerveModulePosition(backRight.getDrivePosition(), getInitRotation2d(backRight))};

        odometer =  new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            // get rotation2d maybe instead of manually setting it? Is the Gyro Relative?
           gyro.getRotation2d(), initpos);
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                backRight.resetEncoders();
                backLeft.resetEncoders();
                frontLeft.resetEncoders();
                frontRight.resetEncoders();
                stopModules();
            } catch (Exception e) {
            }
        }).start();
        
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.5, 0.0, 0.0), // Rotation PID constants
                    4.0, // Max module speed, in m/s
                    0.3556, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public void zeroHeading() {
        gyro.reset();
    }  

    public double getHeading() {
        //return  Math.IEEEremainder(gyro.getAngle(), 360);
        return -1 * (gyro.getYaw() - 90);
        
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetAllEncoders(){
        backRight.resetEncoders();
        backLeft.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
    }

    public Rotation2d getInitRotation2d(SwerveModule swrvMod) {
        return Rotation2d.fromRadians(swrvMod.getAbsoluteEncoderRad());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }


public SwerveModulePosition[] getModulePositions(){
     SwerveModulePosition []currentpos = new SwerveModulePosition[] { new SwerveModulePosition(frontLeft.getDrivePosition(), getInitRotation2d(frontLeft)), new SwerveModulePosition(frontRight.getDrivePosition(), getInitRotation2d(frontRight)), 
            new SwerveModulePosition(backLeft.getDrivePosition(), getInitRotation2d(backLeft)), new SwerveModulePosition(backRight.getDrivePosition(), getInitRotation2d(backRight))};
    return currentpos;
}

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getModulePositions(), pose);
        //figure out swerve module position
    }

    @Override
    public void periodic() {

       
        // Used for Odometry purposes only, does not affect Teleop

        SwerveModulePosition lf = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition()));
        SwerveModulePosition rf = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition()));
        SwerveModulePosition lb = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition()));
        SwerveModulePosition rb = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()));

        odometer.update(getRotation2d(), 
            new SwerveModulePosition[]{
                lf, rf, lb, rb
            });

            SmartDashboard.putNumber("Gyro:", getHeading());
            SmartDashboard.putNumber("Left Front Swerve: ", frontLeft.getTurningPosition());
            SmartDashboard.putNumber("Right Front Swerve: ", frontRight.getTurningPosition());
            SmartDashboard.putNumber("Left Back Swerve: ", backLeft.getTurningPosition());
            SmartDashboard.putNumber("Right Back Swerve: ", backRight.getTurningPosition());

            SmartDashboard.putNumber("Left Front Swerve Absolute: ", frontLeft.getAbsoluteEncoderRad());
            SmartDashboard.putNumber("Right Front Swerve Absolute: ", frontRight.getAbsoluteEncoderRad());
            SmartDashboard.putNumber("Left Back Swerve Absolute: ", backLeft.getAbsoluteEncoderRad());
            SmartDashboard.putNumber("Right Back Swerve Absolute: ", backRight.getAbsoluteEncoderRad());

            SmartDashboard.putNumber("Left Front Drive Position: ", frontLeft.getDrivePosition());
            SmartDashboard.putNumber("Right Front Drive Position: ", frontRight.getDrivePosition());
            SmartDashboard.putNumber("Left Back Drive Position: ", backLeft.getDrivePosition());
            SmartDashboard.putNumber("Right Back Drive Position: ", backRight.getDrivePosition());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveModuleState[] currentStates = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
        publisher.set(currentStates);
        publisher2.set(desiredStates);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        
    
    //arrayPublisher.set(new Pose2d[] {getPose()});

        
        
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        SwerveModuleState[] states = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);


    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);

    }

    public void setWheelState( ){

    }
}