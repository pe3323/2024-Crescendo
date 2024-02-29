package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    
    private Intake intakeSubsystem;

    public IntakeNote( Intake intakeSubsystem ){
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.raise();
    }

    @Override
    public void end(boolean x) {
        //intakeSubsystem.lower();
       
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false; //intakeSubsystem.HasNote();
    }
}
