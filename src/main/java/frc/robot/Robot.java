/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import frc.robot.RobotContainer;
// import frc.robot.commands.AutoCommand;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  //Command autonomousCommand;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double startTime;
  public static boolean retractOnDisabled;
  public static boolean disabled;
  public static RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    //m_autonomous = new AutonomousCommand(m_robotContainer.m_TankDrive, m_robotContainer.getXbox());
    m_robotContainer = new RobotContainer();
    disabled = false;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    System.out.println("Robot Init - NOW");
    
   CameraServer.startAutomaticCapture(0);
   CameraServer.startAutomaticCapture(1);

    // m_robotContainer.m_gyro.calibrate();
    //System.out.println("Calibrate!!");

  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   * 
   * <p>
   * Does barely anything???
   */
  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      //System.out.println("X: " + m_robotContainer.limelight.getX());
      //System.out.println("Y: " + m_robotContainer.limelight.getY());
      //System.out.println("Tag ID is: " + m_robotContainer.limelight.getTagID() + " and pipeline is " + m_robotContainer.limelight.getPipeline());
      //System.out.println("Dist.: " + m_robotContainer.limelight.getDistance());


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //m_robotContainer.m_aimSystem.setPosition(28);
    //m_robotContainer.m_climbSystem.setPosition(0);
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    //RobotContainer.lightSystem.getAllianceColor();
    //startTime = Timer.getFPGATimestamp();
    m_autonomousCommand.schedule();
    //RobotContainer.lightSystem.getAllianceColor();
    //m_robotContainer.autoCommand.schedule();
    //m_autonomous.start();
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    /*
    double time = Timer.getFPGATimestamp();
    System.out.println("Time: " + (time - startTime));

    if ((time - startTime) > 3){
      m_robotContainer.autoCommand.cancel();
    }
    */
    
    // if (((time - startTime) < 10) && ((time-startTime) > 3.2)){
    //   m_robotContainer.autoshoot.schedule();
    // }
    // else{
    //   m_robotContainer.autoshoot.cancel();
    // }

  }

  @Override
  public void teleopInit() {
    //RobotContainer.lightSystem.getAllianceColor();
   }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // System.out.println(m_robotContainer.m_aimSystem.getPosition());
  }
  

  /*
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledInit(){
    disabled = true;
    //System.out.println("disableInit retract " + retractOnDisabled);
  }
    
}
