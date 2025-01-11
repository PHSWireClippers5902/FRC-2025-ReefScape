package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;


public class SingleMotor extends SubsystemBase{
    public XboxController xbox;
    public WPI_TalonSRX myMotor;
    public SingleMotor(XboxController x){
        xbox = x;
        myMotor = new WPI_TalonSRX(5);
    } 
    @Override 
    public void periodic(){
        if (xbox.getAButton()){
            myMotor.set(1);
        }
        else if (xbox.getBButton()){
            myMotor.set(-1);
        }
        else {
            myMotor.set(0);
        }
    }
}
