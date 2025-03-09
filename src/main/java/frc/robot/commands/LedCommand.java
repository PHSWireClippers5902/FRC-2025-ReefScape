package frc.robot.commands;
import java.util.Optional;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;
public class LedCommand extends Command{
      
   public LedSubsystem ledSystem;
   public LedCommand(LedSubsystem m_LedSubSystem){
        ledSystem=m_LedSubSystem;
        addRequirements(ledSystem);
   }

   @Override public void execute(){
         // ledSystem.setParty();
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
         if (ally.get().equals(Alliance.Red) ) {
            ledSystem.setRed();
         }
         else if (ally.get().equals(Alliance.Blue)) {
            ledSystem.setBlue();
         }
      }
      else {
         ledSystem.setParty();
      }
   }
}
