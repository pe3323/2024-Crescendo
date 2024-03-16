package frc.robot;

import java.nio.file.Path;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NeoClimber;
import frc.robot.subsystems.NeoShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.KrakenShooter;
import frc.robot.subsystems.Lighting;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Limelight limelightSubsystem = new Limelight();
        private final Shooter shooterSubsystem = new NeoShooter();
        private final ShooterPivot shooterPivotSubsystem = new ShooterPivot();
        private final Intake intakeSubsystem = new Intake();
        private final Climber leftClimber = new NeoClimber(ClimberConstants.climberID1);
        private final Climber rightClimber = new NeoClimber(ClimberConstants.climberID2);
        private final Lighting lightingSubsystem = new Lighting(0);

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
        public Trigger yShooterButton = shooterJoytick.y();
        public Trigger aShooterButton = shooterJoytick.a();
        public Trigger opLeftBumper = shooterJoytick.leftBumper();
        public Trigger opRightBumper = shooterJoytick.rightBumper();
        public Trigger leftTrigger = shooterJoytick.leftTrigger();
        public Trigger rightTrigger = shooterJoytick.rightTrigger();

        SendableChooser<Command> m_chooser = new SendableChooser<>();

        
        
        public RobotContainer() {

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoytick.getLeftX(),
                                () -> -driverJoytick.getLeftY(),
                                () -> -driverJoytick.getRightX(),
                                () -> true));

                NamedCommands.registerCommand("autoTarget",
                                new AutoTarget(limelightSubsystem, swerveSubsystem, shooterPivotSubsystem));
                
                NamedCommands.registerCommand("shoot",
                                new AutoShoot(intakeSubsystem, shooterSubsystem));

                NamedCommands.registerCommand("intake",
                                new IntakeNote(intakeSubsystem, lightingSubsystem));   
                NamedCommands.registerCommand("aim",
                                new AutoAim(limelightSubsystem, shooterPivotSubsystem, lightingSubsystem));             
                configureButtonBindings();

                // Add commands to the autonomous command chooser
                m_chooser.setDefaultOption("T", new PathPlannerAuto("T"));
                m_chooser.addOption("M", new PathPlannerAuto("M"));
                m_chooser.addOption("B", new PathPlannerAuto("B"));
                m_chooser.addOption("TTMB", new PathPlannerAuto("TTMB"));
                m_chooser.addOption("BBMT", new PathPlannerAuto("BBMT"));
                m_chooser.addOption("MMTB", new PathPlannerAuto("MMTB"));
                m_chooser.addOption("BB", new PathPlannerAuto("BB"));
                m_chooser.addOption("TT", new PathPlannerAuto("TT"));
                m_chooser.addOption("MM", new PathPlannerAuto("MM"));
                m_chooser.addOption("TT1", new PathPlannerAuto("TT1"));
                m_chooser.addOption("BB5", new PathPlannerAuto("BB5"));
                m_chooser.addOption("BB4", new PathPlannerAuto("BB4"));
                m_chooser.addOption("BB3", new PathPlannerAuto("BB3"));
                m_chooser.addOption("TTM", new PathPlannerAuto("TTM"));
                m_chooser.addOption("MMT", new PathPlannerAuto("MMT"));
                m_chooser.addOption("MMB", new PathPlannerAuto("MMB"));
                m_chooser.addOption("BBM", new PathPlannerAuto("BBM"));
                m_chooser.addOption("MM1", new PathPlannerAuto("MM1"));
                m_chooser.addOption("MM2", new PathPlannerAuto("MM2"));
                m_chooser.addOption("MM3", new PathPlannerAuto("MM3"));
                m_chooser.addOption("TTM1", new PathPlannerAuto("TTM1"));
                m_chooser.addOption("TTM2", new PathPlannerAuto("TTM2"));
                m_chooser.addOption("TTM3", new PathPlannerAuto("TTM3"));
                m_chooser.addOption("TT123-", new PathPlannerAuto("TT123-"));
                
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

                yButton.onTrue(new AutoTarget(limelightSubsystem, swerveSubsystem, shooterPivotSubsystem).andThen(new AutoAim(limelightSubsystem, shooterPivotSubsystem, lightingSubsystem)));

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

                bShooterButton.whileTrue(new Shoot(intakeSubsystem, shooterSubsystem, lightingSubsystem) 
                );

                xShooterButton.onTrue(new IntakeNote(intakeSubsystem, lightingSubsystem));

                //yShooterButton.onTrue(new AutoAim(limelightSubsystem, shooterPivotSubsystem));

                aShooterButton.onTrue(new Command() {
                        @Override
                        public void execute() {
                                shooterPivotSubsystem.setPosition(5);
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

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
                return m_chooser.getSelected();
              }
}