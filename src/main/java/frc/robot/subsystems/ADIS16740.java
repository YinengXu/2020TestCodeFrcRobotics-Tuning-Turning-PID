/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;
import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ADIS16740 extends SubsystemBase {
  /**
   * Creates a new ADIS16740.
   */
  public static ADIS16470_IMU gyro = new ADIS16470_IMU();
  
  public ADIS16740() {
    
    //gyro.configCalTime(ADIS16470CalibrationTime._16s);
   //gyro.calibrate();
  }
  public void close(){
    gyro.close();
    
  }
  public void getTemperature(){
  
  }
  
  public void calibrate(){
    gyro.calibrate();
  }
  public void resetAngle(){
    gyro.reset();
  }
  public double angle(){
    return gyro.getAngle();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
