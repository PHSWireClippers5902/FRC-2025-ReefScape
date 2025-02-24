package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmHookConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command{
    
    public Elevator elevator;
    public XboxController xbox;
    public ElevatorCommand(Elevator elevatorSystem, XboxController m_xbox){
        xbox = m_xbox;
        elevator = elevatorSystem;
        addRequirements(elevator);

    }
    @Override
    public void execute() {
        //execute order 66
        SmartDashboard.putNumber("Arm", elevator.getArmPosition());
        SmartDashboard.putNumber("Wrist", elevator.getWristPosition());
        SmartDashboard.putNumber("Intake", elevator.getIntakePosition());
        SmartDashboard.putNumber("Extendo", elevator.getExtendoPosition());
        if (xbox.getAButton()){
            // elevator.powerMovement(new double[]{-22,0,-10});
            elevator.powerMovement(new double[]{-35,0,0});
        }
        else if (xbox.getBButton()){
            elevator.powerMovement(ArmHookConstants.intakePosUp);
        }
        else if (xbox.getRightBumperButton()){
            elevator.intake(true);
        }
        else if (xbox.getLeftBumperButton()){
            elevator.intake(false);
        }
        else if (xbox.getXButton()){
            elevator.wristExtendoMovement(new double[]{0,-8});
        }
        else if (xbox.getYButton()){
            elevator.wristExtendoMovement(new double[]{0,0});
        }
        else {
            elevator.moveIntake_ONLY_IN_EMERGENCIES(0);
        }


    }
}
