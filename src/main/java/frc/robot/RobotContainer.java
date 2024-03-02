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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NeoClimber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.KrakenShooter;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Limelight limelightSubsystem = new Limelight();
        private final Shooter shooterSubsystem = new KrakenShooter();
        private final ShooterPivot shooterPivotSubsystem = new ShooterPivot();
        private final Intake intakeSubsystem = new Intake();
        private final Climber leftClimber = new NeoClimber(ClimberConstants.climberID1);
        private final Climber rightClimber = new NeoClimber(ClimberConstants.climberID2);

        // private final Joystick driverJoytick = new
        // Joystick(OIConstants.kDriverControllerPort);

        private final CommandXboxController driverJoytick = new CommandXboxController(OIConstants.kDriverControllerPort);
       public Trigger aButton = driverJoytick.a();
        public Trigger leftBumper = driverJoytick.leftBumper();
       public Trigger rightBumper = driverJoytick.rightBumper();
        public Trigger xButton = driverJoytick.x();
       public Trigger yButton = driverJoytick.y();

        private final CommandXboxController shooterJoytick = new CommandXboxController(OIConstants.kShooterControllerPort);
        public Trigger bShooterButton = shooterJoytick.b();
        public Trigger xShooterButton = shooterJoytick.x();
        public Trigger opLeftBumper = shooterJoytick.leftBumper();
        public Trigger opRightBumper = shooterJoytick.rightBumper();
        public Trigger leftTrigger = shooterJoytick.leftTrigger();
        public Trigger rightTrigger = shooterJoytick.rightTrigger();

        public RobotContainer() {

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoytick.getLeftX(),
                                () -> -driverJoytick.getLeftY(),
                                () -> -driverJoytick.getRightX(),
                                () -> true));

                NamedCommands.registerCommand("autoAim",
                                new AutoTarget(limelightSubsystem, swerveSubsystem, shooterPivotSubsystem));

                configureButtonBindings();

        }

        private void configureButtonBindings() {
                aButton.onTrue(new Command() {
                        @Override
                        public void execute() {
                                swerveSubsystem.resetAllEncoders();
                                swerveSubsystem.zeroHeading();
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                yButton.onTrue(new AutoTarget(limelightSubsystem, swerveSubsystem, shooterPivotSubsystem));

                leftBumper.onTrue(new Command() {
                        @Override
                        public void execute() {
                                if (SwerveJoystickCmd.SpeedModifier != 3) {
                                        SwerveJoystickCmd.SpeedModifier++;
                                }
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                rightBumper.onTrue(new Command() {
                        @Override
                        public void execute() {
                                if (SwerveJoystickCmd.SpeedModifier != 0) {
                                        SwerveJoystickCmd.SpeedModifier--;
                                }
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                xButton.onTrue(new Command() {
                        @Override
                        public void execute() {
                                limelightSubsystem.getValues();
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                bShooterButton.whileTrue(new Command() {
                        @Override
                        public void execute() {
                                shooterSubsystem.setSpeed(0.8);

                        }

                        @Override
                        public void end(boolean x) {
                                shooterSubsystem.stop();
                        }

                        public boolean isFinished() {
                                return false;
                        }
                });

                xShooterButton.onTrue(new IntakeNote(intakeSubsystem));

                opLeftBumper.whileTrue(new Command() {
                        public void execute() {
                               
                                leftClimber.lower();

                        }

                        public void end(boolean x) {
                                
                                leftClimber.stop();
                        }

                        public boolean isFinished() {
                                return false;
                        }
                });

                opRightBumper.whileTrue(new Command() {
                        public void execute() {
                                rightClimber.lower();
                                

                        }

                        public void end(boolean x) {
                                rightClimber.stop();
                                
                        }

                        public boolean isFinished() {
                                return false;
                        }
                });

                leftTrigger.whileTrue(new Command() {
                        public void execute() {
                                
                                leftClimber.raise();

                        }

                        public void end(boolean x) {
                               
                                leftClimber.stop();
                        }

                        public boolean isFinished() {
                                return false;
                        }
                }); 

                rightTrigger.whileTrue(new Command() {
                        public void execute() {
                                
                                rightClimber.raise();

                        }

                        public void end(boolean x) {
                               
                                rightClimber.stop();
                        }

                        public boolean isFinished() {
                                return false;
                        }
                }); 

        }

        public Command getAutonomousCommand() {
                return new PathPlannerAuto("Auto1");
        }
}