package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Translation2d;
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
        if (myXbox.getLeftStickButton()){
            mySwerve.myGyro.reset();
        }
        if (myXbox.getLeftTriggerAxis() > 0.2){
            if (hasProcced){
                hasProcced = false;
                fieldRelative = !fieldRelative;
                
            }
        }
        else {
            hasProcced = true;
        }
        if (myXbox.getRightTriggerAxis() > 0.2){
            normMax = 0.4;
            rotMax = 0.2;
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
                double[] testSpeeds = getLimelightAlignmentSpeeds(0,20);
                mySwerve.drive(testSpeeds[0],0,testSpeeds[1],false,0.1,new Translation2d(0,0));
            }
        }
        else {
            
            mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02, customCenter);
        }

        SmartDashboard.putNumber("Technical speed X before anything is done.",getLimelightAlignmentSpeeds(0,20)[0]);
        SmartDashboard.putNumber("Technical speed Y before anything is done.",getLimelightAlignmentSpeeds(0,20)[1]);
        SmartDashboard.putNumber("Distance from goal", llvalues.getInchesFromGoal());
        // mySwerve.m_frontLeft.steeringController.set(ControlMode.Position,135 * 4096 / 360);
        // mySwerve.m_frontRight.steeringController.set(ControlMode.Position,45 * 4096 / 360);
        // mySwerve.m_backLeft.steeringController.set(ControlMode.Position,-135 * 4096 / 360);
        // mySwerve.m_backRight.steeringController.set(ControlMode.Position,-45 * 4096 / 360);

        // mySwerve.m_frontLeft.powerController.set(0.1);
        // mySwerve.m_frontRight.powerController.set(0.1);
        // mySwerve.m_backLeft.powerController.set(0.1);
        // mySwerve.m_backRight.powerController.set(0.1);

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
