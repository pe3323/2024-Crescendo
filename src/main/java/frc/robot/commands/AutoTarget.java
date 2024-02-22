package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTarget extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Limelight limelightSubsystem;
    private final ShooterPivot shooterPivotSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public static double SpeedModifier = 0;
        public AutoTarget(Limelight AprilTagSeen, SwerveSubsystem AprilTagID, ShooterPivot shooterPivot) 
        {        
                this.swerveSubsystem=AprilTagID;
                this.limelightSubsystem=AprilTagSeen;
                this.shooterPivotSubsystem=shooterPivot;
                this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem, limelightSubsystem, shooterPivot);




        }
    @Override  
      public void execute(){
  
        double xSpeed = 0.0;
        double ySpeed = 0.0;
        double percentError = Math.abs(limelightSubsystem.getTx()/27.0);
        double turningSpeed = 0;
        if (percentError >= 0.5) {
          turningSpeed = limelightSubsystem.getTx() > 0? -1.0 : 1.0;
        }
        else if (percentError >= 0.15){
          turningSpeed = limelightSubsystem.getTx() > 0? -.5 : .5;
        }
        else if (percentError >= 0.02){
          turningSpeed = limelightSubsystem.getTx() > 0? -.30 : .30;
        }

        SmartDashboard.putNumber("AutoTarget Turn Speed", turningSpeed);
        //PIDController turnController = new PIDController(xSpeed, ySpeed, turningSpeed);
        //turnController.calculate(swerveSubsystem.getHeading(), swerveSubsystem.getHeading() + limelightSubsystem.getTx());
        if (isFinished()) {
            SmartDashboard.putBoolean("AutoTarget Finished", true);
    return;
}


        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        SmartDashboard.putNumber("Turning Speed", turningSpeed);
        
            // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

      }
@Override
      public boolean isFinished(){

        return limelightSubsystem.getClosestAprilTag() == -1
||         Math.abs(limelightSubsystem.getTx()/27.0) <  0.02;
                         
            //if tx value > .5, stop. otherwise keep going   

          // Math.abs(limelightSubsystem.getTx()) <  0.7;
        
      }





}