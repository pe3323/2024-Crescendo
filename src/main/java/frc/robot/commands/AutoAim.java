// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPivot;


/** An example command that uses an example subsystem. */
public class AutoAim extends Command {

private final Limelight limelightSubsystem;
private final ShooterPivot shooter;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAim(Limelight limelightSubsystem, ShooterPivot shooter) {

    this.limelightSubsystem = limelightSubsystem;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelightSubsystem, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   double targetPosition= (Math.tanh(ShooterConstants.goalHeight/limelightSubsystem.getDistanceToTag(4)) - 40.81) * ShooterConstants.rotPerDegree;

    shooter.setPosition(targetPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
