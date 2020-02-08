/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ADIS16740;
import frc.robot.subsystems.DriveTrain;


public class TurningPID extends CommandBase {
  DriveTrain trains;
  ADIS16740 shoot;
  double angs;
  public TurningPID(DriveTrain train, ADIS16740 shooty, double ang) {
    shoot = shooty;
    trains = train;
    angs = ang;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  double time = 0;
  @Override
  public void initialize() {
    time = 0;
    angs= SmartDashboard.getNumber("Angle",0.0);
  }

int i = 1;
double[] vel = {0,0,0,0};
boolean finish;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (time==0){
      finish = false;
      time = System.currentTimeMillis();
    }
    trains.setTurn((angs-shoot.angle())*70);
    trains.setVelocity();
    SmartDashboard.putNumber("Robot ang", shoot.angle());
    SmartDashboard.putNumber("target",angs);
    if (Math.abs(angs - shoot.angle())<0.5){
      finish = true;
    }
    SmartDashboard.putBoolean("end", finish);
    vel[0] = trains.reportLeftEncoder();
    vel[1] = trains.returnLeftVelocity();
    vel[2] = trains.reportRightEncoder();
    vel[3] = trains.returnRightVelocity();
    SmartDashboard.putNumberArray("Vel", vel);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
     trains.arcadeDrive(0, 0);
    trains.setBrake();
    SmartDashboard.putNumber("Efficiency", System.currentTimeMillis()-time);
    time = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finish){
      return true;
    }
    return false;
  }
}
