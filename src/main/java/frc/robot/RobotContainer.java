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
  //create Commands
  // public final Swerve m_swerve = new Swerve();
  public final SystemIdentificationSubsystem sysidSubsystem = new SystemIdentificationSubsystem(xbox);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  // public final SwerveCommand swerveCommand = new SwerveCommand(xbox, m_swerve);
  //Default Constructor
  public RobotContainer(){
    // m_swerve.setDefaultCommand(swerveCommand);
    // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_swerve));    
  }

  private void configureButtonBindings(){ 
    //does nothing 
  }
  
  //for some reason it is important............. idk why
  public XboxController getXbox() {
    return xbox;
  }
  //gets command to run in autonomous
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}