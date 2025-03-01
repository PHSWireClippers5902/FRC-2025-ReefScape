package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLightValues;
import frc.robot.subsystems.Swerve;

public class AutonomousCommand extends Command{
    public Timer timer;
    public Elevator elevator;
    public LimeLightValues llvalues;

    public Swerve swerveSystem;

    public AutonomousCommand(Swerve mySwerve, Elevator myElevator, LimeLightValues myLimeLightValues){
        //stub for initialization
        swerveSystem = mySwerve;
        elevator = myElevator;
        llvalues = myLimeLightValues;
        addRequirements(elevator);
        addRequirements(llvalues);
        addRequirements(swerveSystem);
        timer = new Timer();

    
    }
    
    @Override
    public void initialize() {
        //init command, stub
        timer.reset();
    }

    @Override
    public void execute() {
        //auto sequence enable, stub
        // System.out.println("Hi bro");

        //mapping center

        //if time less than 4, go to about where LL is. 
        //align, then, move right/left for a small period.
        //move claw up to the highest position using PID
        //place 
        //lower and travel to pick up station.

        if (timer.get() < 4){
            swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        }




        if (timer.get() < 5){
            
            System.out.println("First Stage + " + timer.get());
            swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        }
        else if (timer.get() < 6){
            System.out.println("Second Stage + " + timer.get());
            swerveSystem.drive(0,0,-0.1,false,0.1,new Translation2d(0,0));
        }
        else if (timer.get() < 7){
            System.out.println("Third Stage + " + timer.get());
            swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0)); 
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        //end
        System.out.println("Ended.");
    }

    

}
