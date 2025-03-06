package frc.robot;

//Spark Imports

import edu.wpi.first.wpilibj.Ultrasonic;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.subsystems.*;
import frc.robot.commands.*;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.subsystems.AccelerometerSystem;

public class RobotContainer {
  // create inputs
  public final XboxController xbox = new XboxController(0);
  public final XboxController xbox2 = new XboxController(1);
  public final Joystick joystick = new Joystick(1);
  //create Commands
  public final Elevator m_elevator = new Elevator();
  public final Swerve m_swerve = new Swerve();
  public final Lift m_lift = new Lift();
  public final LimeLightValues m_values = new LimeLightValues();
  // public final SystemIdentificationSubsystem sysidSubsystem = new SystemIdentificationSubsystem(xbox);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public final ElevatorCommand m_elevatorCommand = new ElevatorCommand(m_elevator, xbox, xbox2);
  public final SwerveCommand swerveCommand = new SwerveCommand(xbox, m_swerve, m_values);
  public final LiftCommand liftCommand = new LiftCommand(xbox,xbox2, m_lift);
  //Default Constructor
  public RobotContainer(){
    m_elevator.setDefaultCommand(m_elevatorCommand);
    m_swerve.setDefaultCommand(swerveCommand);
    m_lift.setDefaultCommand(liftCommand);
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_swerve, m_elevator, m_values));    
  }

  private void configureButtonBindings(){ 
    //does nothing 
  }
  
  //for some reason it is important............. idk why
  public XboxController getXbox() {
    return xbox;
  }

  public Joystick getJoystick() {
    return joystick;
  }
  //gets command to run in autonomous
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}