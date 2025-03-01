package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class LiftCommand extends Command{
    public Lift lift;
    public XboxController xbox;
    public LiftCommand(XboxController m_xbox, Lift m_lift){
        xbox = m_xbox;
        lift = m_lift;
        addRequirements(lift);
    }
    //execute order 66
    @Override
    public void execute() {
        // if (xbox.getPOV() == 90){
        //     lift.moveLift(0.2);
        // }
        // else if (xbox.getPOV() == 270){
        //     lift.moveLift(-0.2);
        // }
        // else {
        //     lift.moveLift(0);
        // }
    }
}
