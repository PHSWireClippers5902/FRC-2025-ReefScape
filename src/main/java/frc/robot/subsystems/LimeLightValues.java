package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class LimeLightValues extends SubsystemBase{
    //!!IMPORTANT!! ONLY WORKS ON ROBOTS WITH A LIMELIGHT. IF THEY DON't HAVE A LIMELIGHT, then this is most likely commented out so u never have to check it out. 
    LimelightHelpers helpers = new LimelightHelpers();

    //networktable for getting data
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    //constants that are used in calculatino. 
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 3.5;
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 19.5;
    // distance from the target to the floor
    double goalHeightInches = 12.25;
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    
    double angleToGoalRadians = (angleToGoalDegrees * Math.PI) / 180.0;
    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches-limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);

    

    public LimeLightValues(){
        //grabs instances. 
        NetworkTableInstance instance = table.getInstance();
    }
    //get feducial id
    public double getFeducialID(){
        return LimelightHelpers.getFiducialID("limelight");
    }
    

    public double getTx(){
        double x = tx.getDouble(0.0);
        return x;
    
    }
    public double getTv(){
        double v = tv.getDouble(0.0);
        return v;

    }
    public double getTy(){
        double y = ty.getDouble(0.0);
        return y;
    
    }
    public double getTa(){
        double area = ta.getDouble(0.0);
        return area;
    
    }
    //returns inches from goal
    public double getInchesFromGoal(){
        //gets the inches from goal, duh :0
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        ty = table.getEntry("ty");
        targetOffsetAngle_Vertical = ty.getDouble(0.0);

        angleToGoalDegrees = targetOffsetAngle_Vertical - limelightMountAngleDegrees;
        angleToGoalRadians = (angleToGoalDegrees * Math.PI) / 180.0;

        //calculate distance
        distanceFromLimelightToGoalInches = Math.abs((limelightLensHeightInches - goalHeightInches) / Math.tan(angleToGoalRadians));
        
        return distanceFromLimelightToGoalInches;
    }
    
}
