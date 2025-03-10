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
