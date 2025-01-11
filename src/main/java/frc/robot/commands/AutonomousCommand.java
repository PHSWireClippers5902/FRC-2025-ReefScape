package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutonomousCommand extends Command{
    public Swerve swerveSystem;
    public AutonomousCommand(Swerve mySwerve){
        //stub for initialization
        swerveSystem = mySwerve;
        addRequirements(swerveSystem);
    
    }
    
    @Override
    public void initialize() {
        //init command, stub
    }

    @Override
    public void execute() {
        //auto sequence enable, stub
    }

    @Override
    public void end(boolean interrupted) {
        //end
    }

    

}
