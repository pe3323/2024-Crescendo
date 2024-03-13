// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoShoot
 extends CommandBase {

  
  private final Shooter shooter;
  private final Intake intake;
  private Timer timer;
  /**
   * Creates a new AutoShoot
   * .
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot
  (Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(0.9);
                                

    if (shooter.getRPM() > 5200) { 
            intake.raise();
            timer.start();

    }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


    intake.stop();
    shooter.stop();
    // need to stop the shooter, intake
    // and stop the timer.

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
    
  }
}
