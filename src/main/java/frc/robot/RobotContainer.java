package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Limelight limelightSubsystem = new Limelight();
    private final Shooter shooterSubsystem = new Shooter();
    private final ShooterPivot shooterPivotSubsystem = new ShooterPivot();
    private final Intake intakeSubsystem = new Intake();
    
    //private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);
        Trigger aButton = new JoystickButton(driverJoytick, XboxController.Button.kA.value);
        Trigger leftBumper = new JoystickButton(driverJoytick, XboxController.Button.kLeftBumper.value);
        Trigger rightBumper = new JoystickButton(driverJoytick, XboxController.Button.kRightBumper.value);
        Trigger xButton = new JoystickButton(driverJoytick, XboxController.Button.kX.value);
        Trigger yButton = new JoystickButton(driverJoytick, XboxController.Button.kY.value);
        

    private final XboxController shooterJoytick = new XboxController(OIConstants.kShooterControllerPort);   
        Trigger bShooterButton = new JoystickButton(shooterJoytick, XboxController.Button.kB.value);
        Trigger xShooterButton = new JoystickButton(shooterJoytick, XboxController.Button.kX.value);

    public RobotContainer() {
       
       
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getLeftX(),
                () -> -driverJoytick.getLeftY(),
                () -> -driverJoytick.getRightX(),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
                ));

        NamedCommands.registerCommand("autoAim", new AutoTarget(limelightSubsystem, swerveSubsystem, shooterPivotSubsystem));
        

        configureButtonBindings();

        
    }

    
    

    private void configureButtonBindings() {  
        aButton.onTrue( new Command() {
                @Override
                public void execute() {
                        swerveSubsystem.resetAllEncoders();  
                }
        } );

        yButton.onTrue( new AutoTarget(limelightSubsystem, swerveSubsystem, shooterPivotSubsystem) );

        leftBumper.onTrue( new Command() {
                @Override
                public void execute() {
                if (SwerveJoystickCmd.SpeedModifier != 3) {
                        SwerveJoystickCmd.SpeedModifier++;     
                }
                }
                public boolean isFinished() {return true;}
        } );

        rightBumper.onTrue( new Command() {
                @Override
                public void execute() {
                if (SwerveJoystickCmd.SpeedModifier != 0) {
                        SwerveJoystickCmd.SpeedModifier--;
                }
                }
                public boolean isFinished() {return true;}
        } );

                xButton.onTrue( new Command() {
                @Override
                public void execute() {
                        limelightSubsystem.getValues();
                }
                public boolean isFinished() {return true;}
        } );

                bShooterButton.whileTrue( new Command() {
                @Override
                public void execute() {
                        shooterSubsystem.setSpeed(0.8);

                }
                @Override
                public void end(boolean x) {
                        shooterSubsystem.stop();
                }
                public boolean isFinished() {
                        return false; }
        } );

        xShooterButton.whileTrue( new Command() {
                @Override
                public void execute(){
                        intakeSubsystem.raise();
                }

                @Override
                public void end(boolean x) {
                        intakeSubsystem.stop();
                }

                public boolean isFinished() {
                        //intakeSubsystem.stop();
                        return false;}
        } );
}


    public Command getAutonomousCommand() {
       return new PathPlannerAuto("Auto1");
    }
}