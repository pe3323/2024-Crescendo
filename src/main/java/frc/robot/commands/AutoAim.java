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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int targetTag = 4;
    var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Red){
          targetTag = 4;
      }
      else{
          targetTag = 7;
      }

  double targetRadians = Math.atan(ShooterConstants.goalHeight/(limelightSubsystem.getDistanceToTag(targetTag) - 9 ));

  double targetAngle = targetRadians * (180/Math.PI);
  SmartDashboard.putNumber("target angle", targetAngle);

  if (targetAngle<65 && targetAngle>38){

   double targetPosition= ( targetAngle - 38) / ShooterConstants.degreePerRot;

    shooter.setPosition(targetPosition);
    SmartDashboard.putNumber("targetPosition", targetPosition);

    lighting.setSolidColor(247, 191, 235);
  }

  else if (targetAngle<= 38){

    shooter.setPosition(0.0);

  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lighting.setSolidColor (160,0,255);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }
}
