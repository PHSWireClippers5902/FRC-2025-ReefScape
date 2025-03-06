package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class LiftCommand extends Command{
    public Lift lift;
    public XboxController xbox;
    public XboxController xbox2;
    public LiftCommand(XboxController m_xbox, XboxController m_xbox2, Lift m_lift){
        xbox = m_xbox;
        xbox2 = m_xbox2;
        lift = m_lift;
        addRequirements(lift);
    }
    //execute order 66
    @Override
    public void execute() {
        if (xbox2.getPOV() == 90){
            lift.moveLift(-0.2);
        }
        else if (xbox2.getPOV() == 270){
            lift.moveLift(0.2);
        }
        else {
            lift.moveLift(0);
        }
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
