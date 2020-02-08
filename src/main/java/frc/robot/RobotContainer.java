/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ResetNavx;
import frc.robot.commands.TurningPID;
import frc.robot.commands.Update;
import frc.robot.commands.finisher;
//import frc.robot.commands.LimeLight;
import frc.robot.subsystems.ADIS16740;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.basanShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ADIS16740 shooty = new ADIS16740();
  private final DriveTrain trains = new DriveTrain();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Limelight lime = new Limelight();

  
  Joystick leftJoystick;
  JoystickButton button1;
  JoystickButton button2;
  JoystickButton button4;
  JoystickButton button3;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    leftJoystick = new Joystick(0);
    button1 = new JoystickButton(leftJoystick,1);
    button2 = new JoystickButton(leftJoystick,2);
    button3 = new JoystickButton(leftJoystick,3);
    button4 = new JoystickButton(leftJoystick,4);
    SmartDashboard.putNumber("PL", 0.00186);
    SmartDashboard.putNumber("IL", 0.00000);
    SmartDashboard.putNumber("DL", 0.0004);
    SmartDashboard.putNumber("PR", 0.00186); 
    SmartDashboard.putNumber("IR", 0.00000);
    SmartDashboard.putNumber("DR", 0.0004);
    SmartDashboard.putNumber("Angle",20);
    SmartDashboard.putNumber("error", 10);
    configureButtonBindings();
  }
  public double leftJoystickX(){
    SmartDashboard.putNumber("left x",leftJoystick.getRawAxis(1));
    return leftJoystick.getRawAxis(1);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button1.whenPressed(new SequentialCommandGroup(new TurningPID(trains, shooty, SmartDashboard.getNumber("Angle", 0.0)), new finisher()));
    button2.whenPressed(new Update(trains));
    button3.whenPressed(new ResetNavx(shooty));  
  }
  public void setCoast(){
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
