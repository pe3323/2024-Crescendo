// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPivot;


/** An example command that uses an example subsystem. */
public class AutoAim extends Command {

private final Limelight limelightSubsystem;
private final ShooterPivot shooter;
private final Lighting lighting;
private  double targetPosition = 0.0;
private int targetTag = -1;
private boolean aimed = false;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAim(Limelight limelightSubsystem, ShooterPivot shooter, Lighting lighting) {

    this.limelightSubsystem = limelightSubsystem;
    this.shooter = shooter;
    this.lighting = lighting;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelightSubsystem, shooter, lighting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     targetTag = 4;
    var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Red){
          targetTag = 4;
      }
      else{
          targetTag = 7;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimed = false;
    double targetRadians = Math
        .atan(ShooterConstants.goalHeight / (limelightSubsystem.getDistanceToTag(targetTag) -19 ));

    double targetAngle = targetRadians * (180 / Math.PI);
    SmartDashboard.putNumber("target angle", targetAngle);

    if (targetAngle < 68 && targetAngle >= 38) {
      aimed = true;
      SmartDashboard.putBoolean("In Range",true);

      targetPosition = (targetAngle - 38) / ShooterConstants.degreePerRot;

      shooter.setPosition(targetPosition);
      SmartDashboard.putNumber("targetPosition", targetPosition);

      lighting.setSolidColor(247, 191, 235);
    }

    else{ // if (targetAngle < 38) {
      targetPosition = 0.0;
      SmartDashboard.putBoolean("In Range",false);
      lighting.setSolidColor(255,222,89);
      shooter.setPosition(0.0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (aimed)
      lighting.setSolidColor (160,0,255);
  }

  // Continue until shooter is in place.
  @Override
  public boolean isFinished() {
    return true;
  }
}
