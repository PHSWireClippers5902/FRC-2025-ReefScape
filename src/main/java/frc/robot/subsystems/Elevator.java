package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ArmHookConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    public IdleMode coastOrClear = IdleMode.kBrake;

    //all wrist initialization
    public SparkMax wrist;
    public SparkMaxConfig wristConfiguration;

    public RelativeEncoder wristEncoder;
    public SparkClosedLoopController wristController;

    public static final double WRIST_LEFT_LIMIT = ArmHookConstants.wrist.MotorLIMITS.leftLimit;
    public static final double WRIST_RIGHT_LIMIT = ArmHookConstants.wrist.MotorLIMITS.rightLimit;


    //all arm initialization
    public SparkMax arm;
    public SparkMaxConfig armConfiguration;
    
    public RelativeEncoder armEncoder;// = hook.getEncoder();
    public SparkClosedLoopController armController;

    public static final double ARM_LEFT_LIMIT = ArmHookConstants.arm.MotorLIMITS.leftLimit;
    public static final double ARM_RIGHT_LIMIT = ArmHookConstants.arm.MotorLIMITS.rightLimit;

    //all intake initialization
    public SparkMax intake;
    public SparkMaxConfig intakeConfiguration;
    
    public RelativeEncoder intakeEncoder;
    public SparkClosedLoopController intakeController;
    
    // public static final double INTAKE_LEFT_LIMIT = ArmHookConstants.intake.MotorLIMITS.leftLimit;
    // public static final double INTAKE_RIGHT_LIMIT = ArmHookConstants.intake.MotorLIMITS.rightLimit;

    //all extendo initialization
    public SparkMax extendo;
    public SparkMaxConfig extendoConfiguration;
    
    public RelativeEncoder extendoEncoder;
    public SparkClosedLoopController extendoController;
    
    public static final double EXTENDO_LEFT_LIMIT = ArmHookConstants.extendo.MotorLIMITS.leftLimit;
    public static final double EXTENDO_RIGHT_LIMIT = ArmHookConstants.extendo.MotorLIMITS.rightLimit;

    //init class
    public Elevator(){
        /*
         * 
         * 
         * ALL ARM CODE
         * 
         */

        //create arm object
        arm = new SparkMax(ArmHookConstants.arm.CANConstant, MotorType.kBrushless);
        //create the arm configuration
        armConfiguration = new SparkMaxConfig();
        //ONCE YOU FINISH TESTING, SET IDLE MODE TO BRAKE (UNCOMMENT THE LINE BELOW)
        armConfiguration.idleMode(coastOrClear);

        armConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        armConfiguration.closedLoop.pid(ArmHookConstants.arm.MotorGAINS.kP,ArmHookConstants.arm.MotorGAINS.kI,ArmHookConstants.arm.MotorGAINS.kD);
        armConfiguration.closedLoop.maxOutput(ArmHookConstants.arm.MotorGAINS.kPeakOutput);
        armConfiguration.inverted(ArmHookConstants.arm.MotorInverted);
        
        arm.configure(armConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armController = arm.getClosedLoopController();
        
        
        armEncoder = arm.getEncoder();
        armEncoder.setPosition(0);
        // hook.configure(hookConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        /*
         * 
         * ALL WRIST CODE 
         * 
         * 
         */
        wrist = new SparkMax(ArmHookConstants.wrist.CANConstant, MotorType.kBrushless);
        //create the arm configuration
        wristConfiguration = new SparkMaxConfig();
        //ONCE YOU FINISH TESTING, SET IDLE MODE TO BRAKE (UNCOMMENT THE LINE BELOW)
        wristConfiguration.idleMode(coastOrClear);

        wristConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristConfiguration.closedLoop.pid(ArmHookConstants.wrist.MotorGAINS.kP,ArmHookConstants.wrist.MotorGAINS.kI,ArmHookConstants.wrist.MotorGAINS.kD);
        wristConfiguration.closedLoop.maxOutput(ArmHookConstants.wrist.MotorGAINS.kPeakOutput);
        wristConfiguration.inverted(ArmHookConstants.wrist.MotorInverted);
        
        wrist.configure(wristConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wristController = wrist.getClosedLoopController();
        
        
        wristEncoder = wrist.getEncoder();
        wristEncoder.setPosition(0);
        
        /*
         * 
         * 
         * ALL INTAKE CODE
         * 
         */
        intake = new SparkMax(ArmHookConstants.intake.CANConstant, MotorType.kBrushless);
        //create the arm configuration
        intakeConfiguration = new SparkMaxConfig();
        //ONCE YOU FINISH TESTING, SET IDLE MODE TO BRAKE (UNCOMMENT THE LINE BELOW)
        intakeConfiguration.idleMode(coastOrClear);
        

        intakeConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeConfiguration.closedLoop.pid(ArmHookConstants.intake.MotorGAINS.kP,ArmHookConstants.intake.MotorGAINS.kI,ArmHookConstants.intake.MotorGAINS.kD);
        intakeConfiguration.closedLoop.maxOutput(ArmHookConstants.intake.MotorGAINS.kPeakOutput);
        intakeConfiguration.inverted(ArmHookConstants.intake.MotorInverted);
        
        intake.configure(intakeConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeController = intake.getClosedLoopController();
        
        
        intakeEncoder = intake.getEncoder();
        intakeEncoder.setPosition(0);
        /*
         * 
         * ALL EXTENDO CODE
         * 
         * 
         */
        extendo = new SparkMax(ArmHookConstants.extendo.CANConstant, MotorType.kBrushless);
        //create the arm configuration
        extendoConfiguration = new SparkMaxConfig();
        //ONCE YOU FINISH TESTING, SET IDLE MODE TO BRAKE (UNCOMMENT THE LINE BELOW)
        extendoConfiguration.idleMode(coastOrClear);

        extendoConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        extendoConfiguration.closedLoop.pid(ArmHookConstants.extendo.MotorGAINS.kP,ArmHookConstants.extendo.MotorGAINS.kI,ArmHookConstants.extendo.MotorGAINS.kD);
        extendoConfiguration.closedLoop.maxOutput(ArmHookConstants.extendo.MotorGAINS.kPeakOutput);
        extendoConfiguration.inverted(ArmHookConstants.extendo.MotorInverted);
        
        extendo.configure(extendoConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extendoController = extendo.getClosedLoopController();
        
        
        extendoEncoder = extendo.getEncoder();
        extendoEncoder.setPosition(0);
        
        

    }
    
    public void changeArmPID(double kP, double kI, double kD){
        SparkMaxConfig newConfiguration = new SparkMaxConfig();
        newConfiguration.idleMode(coastOrClear);

        newConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        newConfiguration.closedLoop.pid(kP,kI,kD);
        newConfiguration.closedLoop.maxOutput(ArmHookConstants.arm.MotorGAINS.kPeakOutput);
        newConfiguration.inverted(ArmHookConstants.arm.MotorInverted);
        
        arm.configure(newConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void changeWristPID(double kP, double kI, double kD){
        SparkMaxConfig newConfiguration = new SparkMaxConfig();
        newConfiguration.idleMode(coastOrClear);

        newConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        newConfiguration.closedLoop.pid(kP,kI,kD);
        newConfiguration.closedLoop.maxOutput(ArmHookConstants.wrist.MotorGAINS.kPeakOutput);
        newConfiguration.inverted(ArmHookConstants.wrist.MotorInverted);
        
        wrist.configure(newConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void changeIntakePID(double kP, double kI, double kD){
        SparkMaxConfig newConfiguration = new SparkMaxConfig();
        newConfiguration.idleMode(coastOrClear);

        newConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        newConfiguration.closedLoop.pid(kP,kI,kD);
        newConfiguration.closedLoop.maxOutput(ArmHookConstants.intake.MotorGAINS.kPeakOutput);
        newConfiguration.inverted(ArmHookConstants.intake.MotorInverted);
        
        intake.configure(newConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void changeExtendoPID(double kP, double kI, double kD){
        SparkMaxConfig newConfiguration = new SparkMaxConfig();
        newConfiguration.idleMode(coastOrClear);

        newConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        newConfiguration.closedLoop.pid(kP,kI,kD);
        newConfiguration.closedLoop.maxOutput(ArmHookConstants.extendo.MotorGAINS.kPeakOutput);
        newConfiguration.inverted(ArmHookConstants.extendo.MotorInverted);
        
        extendo.configure(newConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    
    public void swapArmPID(boolean up){
        if (up){
            SparkMaxConfig newConfiguration = new SparkMaxConfig();
            newConfiguration.idleMode(coastOrClear);
    
            newConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            newConfiguration.closedLoop.pid(ArmHookConstants.arm.MotorGAINS.kP,ArmHookConstants.arm.MotorGAINS.kI,ArmHookConstants.arm.MotorGAINS.kD);
            newConfiguration.closedLoop.maxOutput(ArmHookConstants.arm.MotorGAINS.kPeakOutput);
            newConfiguration.inverted(ArmHookConstants.arm.MotorInverted);
            
            arm.configure(newConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        else {
            SparkMaxConfig newConfiguration = new SparkMaxConfig();
            newConfiguration.idleMode(coastOrClear);

            newConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            newConfiguration.closedLoop.pid(ArmHookConstants.arm.OptionalMotorGAINS.kP,ArmHookConstants.arm.OptionalMotorGAINS.kI,ArmHookConstants.arm.OptionalMotorGAINS.kD);
            newConfiguration.closedLoop.maxOutput(ArmHookConstants.arm.MotorGAINS.kPeakOutput);
            newConfiguration.inverted(ArmHookConstants.arm.MotorInverted);
            
            arm.configure(newConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        
    }

    public void intake(boolean in){
        // intake.set(in ? ArmHookConstants.intakePercentIn : ArmHookConstants.intakePercentOut);
        intake.set(in ? 0.5 : -0.5);
    }
    public void powerMovement(double[] predeterminedPositions){
        setArmReference(predeterminedPositions[0]);
        setWristReference(predeterminedPositions[1]);
        setExtendoReference(predeterminedPositions[2]);
    }
    public void armWristMovement(double[] predeterminedPositions){
        setArmReference(predeterminedPositions[0]);
        setWristReference(predeterminedPositions[1]);
    
    }
    public void wristExtendoMovement(double[] predeterminedPositions){
        setWristReference(predeterminedPositions[0]);
        setExtendoReference(predeterminedPositions[1]);
    
    }
    public void armExtendoMovement(double[] predeterminedPositions){
        setArmReference(predeterminedPositions[0]);
        setExtendoReference(predeterminedPositions[1]);
    
    }
    //ASSUMING PROPER LIMITATIONS: ENSURE LEFT LIMIT AND RIGHT LIMIT AREN'T NEGATIVE OR SOMETHING
    public void setArmReference(double reference){
        reference = (reference < ARM_LEFT_LIMIT) ? ARM_LEFT_LIMIT : reference;
        reference = (reference > ARM_RIGHT_LIMIT) ? ARM_LEFT_LIMIT : reference;
        armController.setReference(reference,ControlType.kPosition);
    }
    public void setWristReference(double reference){
        reference = (reference < WRIST_LEFT_LIMIT) ? WRIST_LEFT_LIMIT : reference;
        reference = (reference > WRIST_RIGHT_LIMIT) ? WRIST_RIGHT_LIMIT : reference;
        wristController.setReference(reference,ControlType.kPosition);
    }
    public void setExtendoReference(double reference){
        reference = (reference < EXTENDO_LEFT_LIMIT) ? EXTENDO_LEFT_LIMIT : reference;
        reference = (reference > EXTENDO_RIGHT_LIMIT) ? EXTENDO_RIGHT_LIMIT : reference;
        extendoController.setReference(reference,ControlType.kPosition);
    }
    //returns position
    public double getArmPosition(){
        return armEncoder.getPosition();
    }
    public double getWristPosition(){
        return wristEncoder.getPosition();
    }
    public double getIntakePosition(){
        return intakeEncoder.getPosition();
    }
    public double getExtendoPosition(){
        return extendoEncoder.getPosition();
    }
    public double[] getPositions(){
        return new double[]{armEncoder.getPosition(),wristEncoder.getPosition(),intakeEncoder.getPosition(),extendoEncoder.getPosition()};
    }
    //locks encoder
    public void encoderLockArm(double position){
        armController.setReference(position, ControlType.kPosition);
    }
    public void encoderLockWrist(double position){
        wristController.setReference(position, ControlType.kPosition);
    }
    public void encoderLockIntake(double position){
        intakeController.setReference(position, ControlType.kPosition);
    }
    public void encoderLockExtendo(double position){
        extendoController.setReference(position,ControlType.kPosition);
    }
    //zero encoder
    public void zeroEncoders(){
        armEncoder.setPosition(0);
        wristEncoder.setPosition(0);
        intakeEncoder.setPosition(0);
        extendoEncoder.setPosition(0);
    }
    public void zeroArm(){armEncoder.setPosition(0);}
    public void zeroWrist(){wristEncoder.setPosition(0);}
    public void zeroIntake(){intakeEncoder.setPosition(0);}
    public void zeroExtendo(){extendoEncoder.setPosition(0);}
    //ONLY IN EMERGENCIES USE THIS: OTHERWISE USE encoderlock or set___reference
    public void moveArm_ONLY_IN_EMERGENCIES(double speed){
        arm.set(speed);
    }
    public void moveWrist_ONLY_IN_EMERGENCIES(double speed){
        wrist.set(speed);
    }
    public void moveIntake_ONLY_IN_EMERGENCIES(double speed){
        intake.set(speed);
    }
    public void moveExtendoONLY_IN_EMERGENCIES(double speed){
        extendo.set(speed);
    }
    
    public String getString(){
        return String.format("Arm: %.2f | Wrist: %.2f | Intake: %.2f | Extendo %.2f",getArmPosition(),getWristPosition(),getIntakePosition(),getExtendoPosition());
    }


}
