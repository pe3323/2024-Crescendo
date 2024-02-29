package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

      
public class IntakeCommandering  extends Command{ 
    public final Intake orangefruitloop;
     public IntakeCommandering (Intake IntakeSubsystem) 
        {      orangefruitloop = IntakeSubsystem;
            addRequirements(orangefruitloop);
        }
       

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        orangefruitloop.raise();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
              orangefruitloop.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}

