package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LedCommand;

public class LedSubsystem extends SubsystemBase {
      // see https://rag3r.frii.site/files/blinkin.pdf to find the pwm values
    
     Spark strip1 = new Spark(9);// servo is used to controll the led strip via pwm as there is no rev blinkin object
     Spark strip2 = new Spark(8);
    
   
        public void setGreen(){
            strip1.set(0.77);
            strip2.set(-0.89);
        }
        public void setRed(){
            strip1.set(0.61);
            strip2.set(0.61);
            // strip2.set(-0.89);
        }
        public void setBlue(){
            strip1.set(0.87);
            // strip2.set(-0.89);
            strip2.set(0.87);
        }
        public void setParty(){
            strip1.set(-0.87);
            strip2.set(-0.87);
        }
        public void setIdle(){
            strip1.set(-0.31);
            strip2.set(-0.31);
        }
        public void setManual(double pwm){
            strip1.set(pwm);
        }

    
}

