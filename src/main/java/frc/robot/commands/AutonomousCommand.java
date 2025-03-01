package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            SmartDashboard.putString("AUTOV", "1");
            swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        }
        else if (timer.get() < 6){
            SmartDashboard.putString("AUTOV", "2");
            swerveSystem.drive(getLimelightAlignmentSpeeds(0, 20)[0],0,getLimelightAlignmentSpeeds(0, 20)[1],false,0.1,new Translation2d(0,0));
        }
        else if (timer.get() < 8){
            SmartDashboard.putString("AUTOV", "3");
            swerveSystem.drive(0,0.1,0,false,0.1,new Translation2d(0,0));
            elevator.powerMovement(new double[]{0,0,0});
        }
        //change wrist but dont change swerve system
        else if (timer.get() < 8.5){
            SmartDashboard.putString("AUTOV", "4");
            elevator.powerMovement(new double[]{0,0,0});
        }
        else if (timer.get() < 9){
            elevator.moveIntake_ONLY_IN_EMERGENCIES(-0.5);
        }
        else if (timer.get() < 10){
            elevator.moveIntake_ONLY_IN_EMERGENCIES(0);
        }
        else if (timer.get() < 14){
            swerveSystem.drive(-0.2,-0.1,0,false,0.1,new Translation2d(0,0));
        }
        else {
            
            swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0));
        }
        




        // if (timer.get() < 5){
            
        //     System.out.println("First Stage + " + timer.get());
        //     swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        // }
        // else if (timer.get() < 6){
        //     System.out.println("Second Stage + " + timer.get());
        //     swerveSystem.drive(0,0,-0.1,false,0.1,new Translation2d(0,0));
        // }
        // else if (timer.get() < 7){
        //     System.out.println("Third Stage + " + timer.get());
        //     swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0)); 
        // }
        
    }
    private double[] getLimelightAlignmentSpeeds(double targetXAngle, double inchesFromGoal){
        double tx = llvalues.getTx(); //get x angle 
        double ty = llvalues.getInchesFromGoal(); //inches from goal
        
        double kx = 0.01;
        double ky = 0.01;

        double xspeed = kx * (tx-targetXAngle);
        double yspeed = ky * (ty - inchesFromGoal);

        xspeed = Math.abs(xspeed) > 0.5 ? 0.5 : xspeed;
        yspeed = Math.abs(yspeed) > 0.5 ? -0.5 : -yspeed;

        return new double[]{xspeed,yspeed};
    }


    @Override
    public void end(boolean interrupted) {
        //end
        System.out.println("Ended.");
    }

    

}
