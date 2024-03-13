// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Shoot extends CommandBase {
  private final Intake intake;
  private final Shooter shooter;
  private final Lighting lighting;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Intake intake, Shooter shooter, Lighting lighting) {
    this.shooter = shooter;
    this.intake = intake;
    this.lighting = lighting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, lighting);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(0.9);
    lighting.setSolidColor(160, 0, 255);

    if (shooter.getRPM() > 5200) {
      intake.raise();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    intake.stop();
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      lighting.setSolidColor(227, 5, 5);
    } else {
      lighting.setSolidColor(62, 62, 255);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
