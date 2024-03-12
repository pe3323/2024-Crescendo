package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lighting;

public class IntakeNote extends Command {
    
    private Intake intakeSubsystem;
    private final Lighting lighting;

    public IntakeNote(Intake intakeSubsystem, Lighting lighting){
        this.intakeSubsystem = intakeSubsystem;
        this.lighting = lighting;

        addRequirements(intakeSubsystem, lighting);
    }

    @Override
    public void execute() {
        intakeSubsystem.raise();
        lighting.setSolidColor(250, 163, 42);
    }

    @Override
    public void end(boolean x) {
        //intakeSubsystem.lower();
       
        intakeSubsystem.stop();
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Red){
            lighting.setSolidColor (227, 5, 5);
        }
        else{
            lighting.setSolidColor (62, 62, 255);
        }
    }

    @Override
    public boolean isFinished(){
    return intakeSubsystem.HasNote();
    }
}
