// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class testSystem extends SubsystemBase {
  public static WPI_TalonFX testTalon1;
  public static WPI_TalonFX testTalon2, testTalon5;
  public static VictorSPX testTalon3, testTalon4;

  // test shooters: 9 and 10
  // test intake: 11
  // test transition motor: 5
  // test drive motors: 1, 2, 3, 4

  private int testTalon1ID = 9;
  private int testTalon2ID = 10;
  private int testTalon3ID = 5;
  private int testTalon4ID = 11;
  private int testTalon5ID = 12;
  /** Creates a new testSystem. */
  public testSystem() {
    testTalon1 = new WPI_TalonFX(testTalon1ID);
    testTalon2 = new WPI_TalonFX(testTalon2ID);
    testTalon3 = new VictorSPX(testTalon3ID);
    testTalon4 = new VictorSPX(testTalon4ID);
    testTalon5 = new WPI_TalonFX(testTalon5ID);

    testTalon2.follow(testTalon1);
    //upper intake motor
    testTalon5.setInverted(true);
    testTalon2.setInverted(true);
    testTalon3.setInverted(true);
  }

  public void testMethod(double value) {
    // testTalon1.set(ControlMode.Velocity, value);
    // testTalon2.set(ControlMode.PercentOutput, value);
    // testTalon3.set(ControlMode.PercentOutput, 0.8);
    // testTalon4.set(ControlMode.PercentOutput, 0.5);
    // testTalon5.set(ControlMode.PercentOutput, value);
    // SmartDashboard.putBoolean("Motor Tested", true);
    // SmartDashboard.putNumber("Motor Test Value", value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
