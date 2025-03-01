package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.EncoderLimits;
import frc.robot.MotorInitialize;
import frc.robot.Constants.ArmHookConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command{
    
    public Elevator elevator;
    public Joystick joystick;
    public XboxController xbox;
    public boolean proccedUpDown = false;
    public boolean goingUp = false;
    public double[] targets = {0,0,0};
    public double armSpeedUp = 0.3;
    public double armSpeedDown = 0.2;
    public double extendoOut = 0.2;
    public double extendoIn = 0.2;
    public boolean downTriggered1 = true;
    public boolean downTriggered4 = true;
    public boolean intakeChanged = false;

    public double intake = 0;
    public double wristSpeed = 0.7;

    public double extendoLimitOverride = 0;

    public double extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;


    public ElevatorCommand(Elevator elevatorSystem, XboxController m_xbox, Joystick m_joystick){
        xbox = m_xbox;
        elevator = elevatorSystem;
        joystick = m_joystick;
        addRequirements(elevator);

    }
    public boolean inLimit(int arlock, double check, MotorInitialize initEvent){
        return (targets[arlock] + check > initEvent.MotorLIMITS.leftLimit && targets[0] + check < initEvent.MotorLIMITS.rightLimit);
    }
    public double k = 30;
    public void whatLimitShouldBeExtendo(){
        
        if (elevator.getArmPosition() > -8.7){
            extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;
        } else {
        extendoLeftLimit =  - k * Math.abs( Math.cos(Units.degreesToRadians(90-55.54+55.54*-elevator.getArmPosition()/18.547564)))* Math.abs( Math.cos(Units.degreesToRadians(90-55.54+55.54*-elevator.getArmPosition()/18.547564)));
        if (targets[2] < extendoLeftLimit){targets[2] = extendoLeftLimit;}
        }
        // if (elevator.getArmPosition() > -10){extendoLeftLimit = ArmHookConstants.extendo.MotorLIMITS.leftLimit;}
    }
    


    @Override
    public void execute() {
        intakeChanged = false;
        //execute order 66
        SmartDashboard.putNumber("Arm", elevator.getArmPosition());
        SmartDashboard.putNumber("Wrist", elevator.getWristPosition());
        SmartDashboard.putNumber("Intake", elevator.getIntakePosition());
        SmartDashboard.putNumber("Extendo", elevator.getExtendoPosition());
        
        //top left blue
        if (joystick.getRawButton(1)){
            if (downTriggered1){
                // downAState();
                downTriggered1 = false;
            }
            intake = 0.5;
            intakeChanged = true;
        }else {downTriggered1 = true;}
        //joystick mid left green
        if (joystick.getRawButton(4)){
            if (downTriggered4){
                // downAState();
                downTriggered4 = false;
            }
            intake = -0.5;
            intakeChanged = true;
        }else {downTriggered4 = true;}
        //joystick bottom left red
        if (joystick.getRawButton(7)){

        }
        
        //goes down

        if (joystick.getY() > 0.3){
            if (inLimit(0,armSpeedDown,ArmHookConstants.arm)){
                // whatLimitShouldBeExtendo();
                targets[0]+=armSpeedDown;
            }
            else {
                // whatLimitShouldBeExtendo();
                targets[0] = ArmHookConstants.arm.MotorLIMITS.rightLimit;
            }
        }
        //goes up
        else if (joystick.getY() < -0.3){
            // whatLimitShouldBeExtendo();
            if (inLimit(0,-armSpeedUp,ArmHookConstants.arm)){
                targets[0]-=armSpeedUp;
            }
            else {
                whatLimitShouldBeExtendo();
                targets[0] = ArmHookConstants.arm.MotorLIMITS.leftLimit;
            }
        }
        else if (xbox.getYButton()){
            if (inLimit(0,-armSpeedUp,ArmHookConstants.arm)){
                targets[0]-=armSpeedUp;
            }
            else {
                whatLimitShouldBeExtendo();
                targets[0] = ArmHookConstants.arm.MotorLIMITS.leftLimit;
            }
        }
        else if (xbox.getAButton()){
            if (inLimit(0,armSpeedDown,ArmHookConstants.arm)){
                // whatLimitShouldBeExtendo();
                targets[0]+=armSpeedDown;
            }
            else {
                // whatLimitShouldBeExtendo();
                targets[0] = ArmHookConstants.arm.MotorLIMITS.rightLimit;
            }
        }
        //wrist movement
        if (joystick.getX() > 0.3){
            if (inLimit(1,wristSpeed,ArmHookConstants.wrist)){
                targets[1]+=wristSpeed;
            }
            else {
                targets[1] = ArmHookConstants.wrist.MotorLIMITS.rightLimit;
            }
            // targets[1]+=wristSpeed;
        }
        else if (joystick.getX() < -0.3){
            if (inLimit(1,-wristSpeed,ArmHookConstants.wrist)){
                targets[1]-=wristSpeed;
            }
            else {
                targets[1] = ArmHookConstants.wrist.MotorLIMITS.leftLimit;
            }
            // targets[1]-=wristSpeed;
        }
        else if (xbox.getBButton()){
            if (inLimit(1,wristSpeed,ArmHookConstants.wrist)){
                targets[1]+=wristSpeed;
            }
            else {
                targets[1] = ArmHookConstants.wrist.MotorLIMITS.rightLimit;
            }
            // targets[1]+=wristSpeed;
        }else if (xbox.getXButton()){
            if (inLimit(1,-wristSpeed,ArmHookConstants.wrist)){
                targets[1]-=wristSpeed;
            }
            else {
                targets[1] = ArmHookConstants.wrist.MotorLIMITS.leftLimit;
            }
            // targets[1]-=wristSpeed;
        }
        //extendo out
        if (joystick.getRawButton(3)){
            if (inLimit(2,-extendoOut,new MotorInitialize(0, null,new EncoderLimits(extendoLeftLimit,0),true))){
                targets[2]-=extendoOut;
            }
            else {
                targets[2] = extendoLeftLimit;
            }
        }
        //extendo in
        else if (joystick.getRawButton(6)){
            if (inLimit(2,extendoIn,ArmHookConstants.extendo)){
                targets[2]+=extendoIn;
            }
            else {
                targets[2] = ArmHookConstants.extendo.MotorLIMITS.rightLimit;
            }
        }
        else if (xbox.getPOV() == 0){
            if (inLimit(2,-extendoOut,new MotorInitialize(0, null,new EncoderLimits(extendoLeftLimit,0),true))){
                targets[2]-=extendoOut;
            }
            else {
                targets[2] = extendoLeftLimit;
            }
        }
        //extendo in
        else if (xbox.getPOV() == 180){
            if (inLimit(2,extendoIn,ArmHookConstants.extendo)){
                targets[2]+=extendoIn;
            }
            else {
                targets[2] = ArmHookConstants.extendo.MotorLIMITS.rightLimit;
            }
        }
        
        
        
       


        // if (xbox.getAButton()){
        //     // elevator.powerMovement(new double[]{-22,0,-10});
        //     // if (elevator.getArmPosition() > -14){
        //     //     targets[0] = ArmHookConstants.rest[0];
        //     //     // elevator.armExtendoMovement(new double[]{ArmHookConstants.rest[0],0});
        //     // }
        //     // else { 
        //     //     targets = ArmHookConstants.rest;
        //     // }
            
        // }
        // else if (xbox.getBButton()){
        //     // elevator.powerMovement(ArmHookConstants.intakePosUp);
        //     targets = ArmHookConstants.intakePosUp;
        // }
        // else if (xbox.getXButton()){
        //     // elevator.powerMovement(ArmHookConstants.stageThree);
        //     targets = ArmHookConstants.stageThree;
        // }
        // else if (xbox.getYButton()){
        //     // elevator.powerMovement(ArmHookConstants.intakePosDown);
        //     targets = ArmHookConstants.intakePosDown;
        //     intake = 0.5;
        //     // elevator.intake(true);
        // }
        if (xbox.getRightBumperButton()){
            // elevator.intake(true);
            intake = 0.5;
            intakeChanged = true;
        }
        else if (xbox.getLeftBumperButton()){
            intake = -0.5;
            // elevator.intake(false);
            intakeChanged = true;
        }
        else {
            // elevator.moveIntake_ONLY_IN_EMERGENCIES(0);
        }

        
        if (!intakeChanged){intake = 0;}



        whatLimitShouldBeExtendo();
        elevator.powerMovement(targets);
        elevator.moveIntake_ONLY_IN_EMERGENCIES(intake);


    }

}
