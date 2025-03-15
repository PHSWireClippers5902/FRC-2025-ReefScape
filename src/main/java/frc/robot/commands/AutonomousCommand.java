package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmExtendoK;
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
    public double extendoLeftLimit;
    public Swerve swerveSystem;
    public double intake;
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
        if (timer.get() < 6){
            if (llvalues.getTv() > 0){
                double infromgoal = timer.get() < 4 ? 35 : 31;
                double[] testSpeeds = getLimelightAlignmentSpeeds(0,infromgoal,0);
                
                swerveSystem.drive(testSpeeds[1],testSpeeds[0],testSpeeds[2],false,0.1,new Translation2d(0,0));
            }
            else {
                swerveSystem.drive(0,0,0,false,0.02, new Translation2d(0,0));
            }
            if (timer.get() > 3){
                
                targets[2]+=calculateDifference(elevator.getExtendoPosition(),-50.881474,0.02);
            }
            if (timer.get() > 3.5){
                targets[1]+=calculateDifference(elevator.getWristPosition(),-22.785591,0.08);
            }
            targets[0]+=calculateDifference(elevator.getArmPosition(),-34.333023,0.01);
        
        }
        else if (timer.get() < 6.3){
            targets[0]+=calculateDifference(elevator.getArmPosition(), -30, 0.01);
            swerveSystem.drive(0.04,0,0,false,0.02, new Translation2d(0,0));
        
        }
        else if (timer.get() < 6.6){
            swerveSystem.drive(0.04,0,0,false,0.02, new Translation2d(0,0));
        }
        else {
            swerveSystem.drive(0,0,0,false,0.02,new Translation2d(0,0));
        }
        
        
        
            // //goes to rest position middle
            // targets[0]+=calculateDifference(elevator.getArmPosition(),ArmHookConstants.intakePosDown[0],ArmExtendoK.arms);
            // targets[2]+=calculateDifference(elevator.getExtendoPosition(),ArmHookConstants.intakePosDown[2],ArmExtendoK.extendos);




        whatLimitShouldBeExtendo();
        elevator.powerMovement(targets);
        elevator.moveIntake_ONLY_IN_EMERGENCIES(intake);
        
    }

    public double k = 30*4;
    public void whatLimitShouldBeExtendo(){
        
        if (elevator.getArmPosition() > -8){
            extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;
        } else {
        extendoLeftLimit =  - k * Math.abs( Math.cos(Units.degreesToRadians(90-55.54+55.54*-elevator.getArmPosition()/18.547564)))* Math.abs( Math.cos(Units.degreesToRadians(90-55.54+55.54*-elevator.getArmPosition()/18.547564)));
        if (targets[2] < extendoLeftLimit){targets[2] = extendoLeftLimit;}
        }
        // if (elevator.getArmPosition() > -10){extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;}
    }

    private double[] getLimelightAlignmentSpeeds(double targetXAngle, double inchesFromGoal, double targetRotation){
        double tx = llvalues.getTx(); //get x angle 
        double ty = llvalues.getInchesFromGoal(); //inches from goal
        double gyroCurrentPosition = swerveSystem.myGyro.getAng().getDegrees();



        double kx = 0.01;
        double ky = 0.04;
        double kr = 0.04;
        //xspeed: turn, 
        //yspeed: forward
        if (Math.abs(ty-inchesFromGoal) < 10){
            ky = 0.02;
        }
        else if (Math.abs(ty-inchesFromGoal) < 1){
            ky = 0;
        }

        if (Math.abs(tx-targetXAngle) < 3){
            kx = 0.003;
        }
        else if (Math.abs(tx-targetXAngle) < 1.5){
            kx = 0;
        }
        if (Math.abs(targetRotation-gyroCurrentPosition) < 5){
            kr = 0.02;
        }
        else if (Math.abs(targetRotation-gyroCurrentPosition) < 1){
            kr = 0;
        }

        double xspeed = kx * (tx-targetXAngle);
        double yspeed = ky * (ty - inchesFromGoal);
        double rotationSpeed = kr * (targetRotation - gyroCurrentPosition);
        
        xspeed = Math.abs(xspeed) > 0.2 ?  xspeed / Math.abs(xspeed) * 0.2 : xspeed;
        yspeed = Math.abs(yspeed) > 0.2 ? yspeed / Math.abs(yspeed) * 0.2 : yspeed;
        rotationSpeed = Math.abs(rotationSpeed) > 0.2 ? rotationSpeed / Math.abs(rotationSpeed) * 0.2 : rotationSpeed; 

        //reverses left, might need to be done aswell
        xspeed = -xspeed;

        // yspeed = 0;
        //turn, forward
        return new double[]{xspeed,yspeed,rotationSpeed};
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
