// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private WPI_TalonFX climber1;
  private WPI_TalonFX climber2;

  /** Creates a new Climber. */
  public Climber() {
    this.climber1 = new WPI_TalonFX(Constants.CLIMBER1);
    this.climber2 = new WPI_TalonFX(Constants.CLIMBER2);
    climber1.setNeutralMode(NeutralMode.Brake);
    climber2.setNeutralMode(NeutralMode.Brake);
    this.climber1.setInverted(true);
    resetEncoders();
    climber2.follow(climber1);
  }

  public void move(double power){
    climber1.set(ControlMode.PercentOutput, power);
    // climber2.set(ControlMode.PercentOutput, power);
  }

  public void resetEncoders(){
    climber1.setSelectedSensorPosition(0);
    climber2.setSelectedSensorPosition(0);
  }

  public double get7(){
    return climber1.getSelectedSensorPosition();
  }

  public double get8(){
    return climber2.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
