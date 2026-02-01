// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
public class Shooter extends SubsystemBase {
  WPI_VictorSPX LaunchOne = new WPI_VictorSPX(0);
  WPI_VictorSPX LaunchTwo = new WPI_VictorSPX(1);
  /** Creates a new ExampleSubsystem. */
  public Shooter() {}

  
 
//shoot ball
  public void fire(){
   LaunchOne.set(1);
    LaunchTwo.set(1);
  }
  //just in case
  public void HalfFire(){
   LaunchOne.set(0.5);
    LaunchTwo.set(0.5);
  }
  public void reversefire(){
   LaunchOne.set(-1);
    LaunchTwo.set(-1);
  }
  //stop shooting
  public void stop(){
   LaunchOne.set(0);
    LaunchTwo.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
