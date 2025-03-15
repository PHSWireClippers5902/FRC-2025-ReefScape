package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightValues;
import frc.robot.subsystems.Swerve;

public class SwerveCommand extends Command {
    // public Joystick myJoystick;
    public XboxController myXbox;
    public Translation2d customCenter;
    public Translation2d constantRight = new Translation2d(0.40411,-0.40411);
    public Translation2d constantLeft = new Translation2d(0.40411,0.40411);
    public Translation2d constantCenter = new Translation2d(0,0);
    public Swerve mySwerve;
    private static final double kDeadband = 0.2;
    public double daDirection = 1;
    private static final double kZedDeadBand = 0.2;
    private static final double kMaxSpeed = Swerve.kMaxSpeed;
    private static final double kMaxAngularSpeed = Swerve.kMaxAngularSpeed;
    public LimeLightValues llvalues;
    public boolean fieldRelative = false;

    public boolean hasProcced = true;
    double xSpeed,ySpeed,rot;
    public SwerveCommand(XboxController xbox, Swerve m_s, LimeLightValues m_v){
        myXbox = xbox;
        mySwerve = m_s;
        llvalues = m_v;
        addRequirements(mySwerve);
        addRequirements(llvalues);
    }


    public double normMax = 0.1;
    public double rotMax = 0.2;

    @Override
    public void execute() {
        

        // if (myJoystick.getRawButton(0)){

        // }
        //getleftbumper()
        if (false){
            xSpeed = -daDirection*applyDeadband(myXbox.getLeftY()) * kMaxSpeed*0.7;
            // double ySpeed = 0,rot = 0;
            // double rot = 0;
            ySpeed = daDirection*applyDeadband(myXbox.getLeftX()) * kMaxSpeed*0.7;
            rot = -applyZedDeadband(myXbox.getRightX()) * kMaxAngularSpeed*0.2;
        }
        else {
            xSpeed = -daDirection*applyDeadband(myXbox.getLeftY()) * kMaxSpeed*normMax;
            // double ySpeed = 0,rot = 0;
            // double rot = 0;
            ySpeed = -daDirection*applyDeadband(myXbox.getLeftX()) * kMaxSpeed*normMax;
            rot = -applyZedDeadband(myXbox.getRightX()) * kMaxAngularSpeed*rotMax;
        }
        SmartDashboard.putNumber("Limelight x: ", llvalues.getTx());
        SmartDashboard.putNumber("Limelight a: ", llvalues.getTa());
        SmartDashboard.putNumber("Limelight y: ", llvalues.getTy());
        SmartDashboard.putNumber("Limelight v: ", llvalues.getTv());
        // if (myXbox.getRightBumper()){
        //     daDirection = -1;
        // }
        // else {
        //     daDirection = 1;
        // }
        // if (myXbox.getYButton()){
        //     mySwerve.resetAll();
        // }
        // if (myXbox.getAButton()){
        //     mySwerve.myGyro.m_gyro.reset();
        // }
        // if (myXbox.getLeftStickButton()){
        //     mySwerve.myGyro.reset();
        // }
        if (myXbox.getRawButton(7)){
            if (hasProcced){
                hasProcced = false;
                fieldRelative = !fieldRelative;
                
            }
        }
        else {
            hasProcced = true;
        }
        if (myXbox.getLeftTriggerAxis() > 0.2){
            normMax = 0.5;
            rotMax = 0.3;
        }
        else if (myXbox.getRightTriggerAxis() > 0.2){
            normMax = 0.05;
            rotMax = 0.05;
        }
        else {
            normMax = 0.1;
            rotMax = 0.2;
        }

        
        

        SmartDashboard.putBoolean("Field Relative: ", fieldRelative);

        // SmartDashboard.putNumber("Front Left Position: " ,mySwerve.m_frontLeft.steeringController.getSelectedSensorPosition(0)*360/4096);
        // SmartDashboard.putNumber("Front Right Position: " ,mySwerve.m_frontRight.steeringController.getSelectedSensorPosition(0)*360/4096);
        // SmartDashboard.putNumber("Back Left Position: " ,mySwerve.m_backLeft.steeringController.getSelectedSensorPosition(0)*360/4096);
        // SmartDashboard.putNumber("Back Right Position: " , mySwerve.m_backRight.steeringController.getSelectedSensorPosition(0)*360/4096);
        if (myXbox.getPOV() == 270){
            customCenter = constantLeft;
        }
        else if (myXbox.getPOV() == 90){
            customCenter = constantRight;
        }
        else {
            customCenter = constantCenter;
        }



        if (myXbox.getLeftStickButton()){
            if (llvalues.getTv() > 0){
                double[] testSpeeds = getLimelightAlignmentSpeeds(0,50,0);
                mySwerve.drive(testSpeeds[1],testSpeeds[0],testSpeeds[2],false,0.1,new Translation2d(0,0));
            }
            else {
                mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02, customCenter);
            }
        }
        else {
            
            mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02, customCenter);
        }
        if (myXbox.getRightStickButton()){
            mySwerve.myGyro.reset();
        }



        SmartDashboard.putNumberArray("Limelight array tid?: ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]));
        SmartDashboard.putNumber("Limelight array tid position: ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6])[0]);
        SmartDashboard.putNumber("Technical speed X before anything is done.",getLimelightAlignmentSpeeds(0,50,0)[0]);
        SmartDashboard.putNumber("Technical speed Y before anything is done.",getLimelightAlignmentSpeeds(0,50,0)[1]);
        SmartDashboard.putNumber("Technical speed Rot before anything is done.",getLimelightAlignmentSpeeds(0,50,0)[2]);
        SmartDashboard.putNumber("Distance from goal", llvalues.getInchesFromGoal());
        SmartDashboard.putNumber("Fidu: ", llvalues.getFeducialID());
        // mySwerve.m_frontLeft.steeringController.set(ControlMode.Position,135 * 4096 / 360);
        // mySwerve.m_frontRight.steeringController.set(ControlMode.Position,45 * 4096 / 360);
        // mySwerve.m_backLeft.steeringController.set(ControlMode.Position,-135 * 4096 / 360);
        // mySwerve.m_backRight.steeringController.set(ControlMode.Position,-45 * 4096 / 360);

        // mySwerve.m_frontLeft.powerController.set(0.1);
        // mySwerve.m_frontRight.powerController.set(0.1);
        // mySwerve.m_backLeft.powerController.set(0.1);
        // mySwerve.m_backRight.powerController.set(0.1);

    }
    private double[] getLimelightAlignmentSpeeds(double targetXAngle, double inchesFromGoal, double targetRotation){
        double tx = llvalues.getTx(); //get x angle 
        double ty = llvalues.getInchesFromGoal(); //inches from goal
        double gyroCurrentPosition = mySwerve.myGyro.getAng().getDegrees();



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

    private double applyDeadband(double value) {
        return Math.abs(value) > kDeadband ? value : 0.0;
    }
    private double applyZedDeadband(double value){
        if (Math.abs(value) > kZedDeadBand){
            value = (Math.abs(value) - kZedDeadBand)*value/Math.abs(value);
            return value;
        }
        else {
            return 0;
        }
    }

}
