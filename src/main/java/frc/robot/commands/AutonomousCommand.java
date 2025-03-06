package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmHookConstants;
import frc.robot.MotorInitialize;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLightValues;
import frc.robot.subsystems.Swerve;

public class AutonomousCommand extends Command{
    public Timer timer;
    public Elevator elevator;
    public LimeLightValues llvalues;
    public double[] targets = new double[]{0,0,0};
    
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
        timer.start();
    }

    @Override
    public void execute() {
        //auto sequence enable, stub
        // System.out.println("Hi bro");



        // if (timer.get() < 2){
        //     targets[0]+=calculateDifference(elevator.getArmPosition(), -8, 0.1);
        //     swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        
        // }
        // else if (timer.get() < 5){
        //     if (llvalues.getTv() > 0){
        //         double[] targetedValues = getLimelightAlignmentSpeeds(0, 20);
        //         swerveSystem.drive(targetedValues[1],0,targetedValues[0],false,0.1,new Translation2d(0,0));
        //     }
        //     else {
                
        //         swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0));
        //     }
        // }
        // else if (timer.get() < 6){
            
        //     swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0));
        //     elevator.moveIntake_ONLY_IN_EMERGENCIES(-0.5);
        // }
        // else if (timer.get() < 7){
        //     elevator.moveIntake_ONLY_IN_EMERGENCIES(0);
        // }
        // elevator.powerMovement(targets);

        if (timer.get() < 6){
            swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        }
        else {
            swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0));
        }

        //mapping center

        //if time less than 4, go to about where LL is. 
        //align, then, move right/left for a small period.
        //move claw up to the highest position using PID
        //place 
        //lower and travel to pick up station.

        // if (timer.get() < 4){
        //     SmartDashboard.putString("AUTOV", "Forward");
        //     swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        // }
        // else if (timer.get() < 5){
        //     SmartDashboard.putString("AUTOV", "Turn");
        //     // swerveSystem.drive(0,0,-0.1,false,0.1,new Translation2d(0,0));
        //     targets[0] += calculateDifference(elevator.getArmPosition(), -10, 0.03);
            
        // }
        // else if (timer.get() < 8){
        //     SmartDashboard.putString("AUTOV", "Align");
        // swerveSystem.drive(getLimelightAlignmentSpeeds(0, 13)[1],0,getLimelightAlignmentSpeeds(0, 13)[0],false,0.1,new Translation2d(0,0));
            
        //     }
        // //deposit
        // // else if (timer.get() < 9){
            
        // //     SmartDashboard.putString("AUTOV", "Forward");
        // //     swerveSystem.drive(0.1,0,0,false,0.1,new Translation2d(0,0));
        // // }
        // else if (timer.get() < 10){
        //     SmartDashboard.putString("AUTOV", "Deposit");
        //     elevator.moveIntake_ONLY_IN_EMERGENCIES(-0.5);
        //     swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0));
        // }

        // else {
            
        //     swerveSystem.drive(0,0,0,false,0.1,new Translation2d(0,0));
        // }
        
        // elevator.powerMovement(targets);



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
        if (llvalues.getTv() > 0){
        double tx = llvalues.getTx(); //get x angle 
        double ty = llvalues.getInchesFromGoal(); //inches from goal
        
        double kx = 0.04;
        double ky = 0.04;
        //xspeed: turn, 
        //yspeed: forward
        double xspeed = kx * (tx-targetXAngle);
        double yspeed = ky * (ty - inchesFromGoal);

        xspeed = Math.abs(xspeed) > 0.2 ?  xspeed / Math.abs(xspeed) * 0.2 : xspeed;
        yspeed = Math.abs(yspeed) > 0.2 ? yspeed / Math.abs(yspeed) * 0.2 : yspeed;
        xspeed = -xspeed;

        // yspeed = 0;
        //turn, forward
        return new double[]{xspeed,yspeed};
        }
        else {
            return new double[]{0,0};
        }
    }


    @Override
    public void end(boolean interrupted) {
        //end
        System.out.println("Ended.");
    }
    public boolean inLimit(int arlock, double check, MotorInitialize initEvent){
        return (targets[arlock] + check > initEvent.MotorLIMITS.leftLimit && targets[arlock] + check < initEvent.MotorLIMITS.rightLimit);
    }
    // public double k = 30;
    // public void whatLimitShouldBeExtendo(){
        
    //     if (elevator.getArmPosition() > -8.5){
    //         extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;
    //     } else {
    //     extendoLeftLimit =  - k * Math.abs( Math.cos(Units.degreesToRadians(90-55.54+55.54*-elevator.getArmPosition()/18.547564)))* Math.abs( Math.cos(Units.degreesToRadians(90-55.54+55.54*-elevator.getArmPosition()/18.547564)));
    //     if (targets[2] < extendoLeftLimit){targets[2] = extendoLeftLimit;}
    //     }
    //     // if (elevator.getArmPosition() > -10){extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;}
    // }
    public double calculateDifference(double currentPos, double targetPos, double kConst){
        // double kConst = 0.01;
        return kConst * (targetPos - currentPos);
    }
    

}
