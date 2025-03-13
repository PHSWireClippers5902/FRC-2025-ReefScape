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
    double limelightMountAngleDegrees = 0;
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
    public double getFeducialID(){
        return helpers.getFiducialID("limelight");
    }
    public void turnOff(){
        
    }

    public double getTx(){
        
        //read values periodically
        double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);
        // double area = ta.getDouble(0.0);
        return x;
    
    }
    public double getTv(){
        //returns tv (some sorta stuff.)
        double v = tv.getDouble(0.0);
        return v;

    }
    public double getTy(){
        
        //read values periodically
        //double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        return y;
    
    }
    public double getTa(){
        
        //read values periodically
        // double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        return area;
    
    }
    // public double getTy(){


    // }
    public double getInchesFromGoal(){
        //gets the inches from goal, duh :0
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        ty = table.getEntry("ty");
        targetOffsetAngle_Vertical = ty.getDouble(0.0);



    // how many degrees back is your limelight rotated from perfectly vertical?
        //limelightMountAngleDegrees = -4; 

    // distance from the center of the Limelight lens to the floor
        //limelightLensHeightInches = 13.25; 

    // distance from the target to the floor
        //goalHeightInches = 18.5; 
        angleToGoalDegrees = targetOffsetAngle_Vertical + limelightMountAngleDegrees;
        //double diffinheight = Math.abs(goalHeightInches - limelightLensHeightInches);
        //angleToGoalDegrees = targetOffsetAngle_Vertical;
        angleToGoalRadians = (angleToGoalDegrees * Math.PI) / 180.0;

        //SmartDashboard.addNumber();
        // SmartDashboard.putNumber()
        // SmartDashboard.putNumber("D2-D1", Math.abs(goalHeightInches - limelightLensHeightInches));
        //SmartDashboard.putNumber("")

        //calculate distance
        distanceFromLimelightToGoalInches = Math.abs((limelightLensHeightInches - goalHeightInches) / Math.tan(angleToGoalRadians));
        // SmartDashboard.putNumber("Tan(Angle)", Math.tan(angleToGoalRadians));
        // SmartDashboard.putNumber("angletoGoalRadian", angleToGoalRadians);
        
        

    //     if (angleToGoalDegrees == -4) {
    //      //   SmartDashboard.putNumber("Distance from LimeLight in Inches: ",0);
    //         return 0;

    //     }
    //     else {

    //    // SmartDashboard.putNumber("Distance from LimeLight in Inches: ",distanceFromLimelightToGoalInches);
    //     return distanceFromLimelightToGoalInches;
    //     }
        return distanceFromLimelightToGoalInches;
    }
    
}
