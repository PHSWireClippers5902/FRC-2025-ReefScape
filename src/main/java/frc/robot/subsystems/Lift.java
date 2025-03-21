package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmHookConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift extends SubsystemBase{
    public IdleMode coastOrClear = IdleMode.kBrake;

    //all lift initialization
    public SparkMax lift;
    public SparkMaxConfig liftConfiguration;
    public Servo liftServo;
    public RelativeEncoder liftEncoder;
    public SparkClosedLoopController liftController;

    public static final double LIFT_LEFT_LIMIT = ArmHookConstants.lift.MotorLIMITS.leftLimit;
    public static final double LIFT_RIGHT_LIMIT = ArmHookConstants.lift.MotorLIMITS.rightLimit;

    public Lift(){
        //lift
        lift = new SparkMax(ArmHookConstants.lift.CANConstant, MotorType.kBrushless);
        liftServo = new Servo(7);
        liftConfiguration = new SparkMaxConfig();
        //ONCE YOU FINISH TESTING, SET IDLE MODE TO BRAKE (UNCOMMENT THE LINE BELOW)
        liftConfiguration.idleMode(coastOrClear);

        liftConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        liftConfiguration.closedLoop.pid(ArmHookConstants.lift.MotorGAINS.kP,ArmHookConstants.lift.MotorGAINS.kI,ArmHookConstants.lift.MotorGAINS.kD);
        liftConfiguration.closedLoop.maxOutput(ArmHookConstants.lift.MotorGAINS.kPeakOutput);
        liftConfiguration.inverted(ArmHookConstants.lift.MotorInverted);
        

        lift.configure(liftConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        liftController = lift.getClosedLoopController();
        
        
        liftEncoder = lift.getEncoder();
        liftEncoder.setPosition(0);

    }
    public void moveLift(double speed){
        lift.set(speed);
    }
    public void ServoUp(){
        // liftServo.setAngle(90);
        liftServo.set(0.8);
    }
    public void ServoDown(){
        // liftServo.setAngle(2);
        liftServo.set(0);
    }
    public void liftDown(){
        liftController.setReference(-69,ControlType.kPosition);
    }
    public void liftUp(){
        liftController.setReference(0,ControlType.kPosition);

    }
    public double getServoAngle() {
        return liftServo.getAngle();
    }

    public void setServoAngle(double angle) {
        liftServo.setAngle(angle);
    }
    public double calculateDifference(double currentAngle, double nextAngle, double kConstant){
        return kConstant*(nextAngle-currentAngle);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Servo Angle", getServoAngle());
        SmartDashboard.putNumber("Lift motor Angle: ", liftEncoder.getPosition());
    }
}
