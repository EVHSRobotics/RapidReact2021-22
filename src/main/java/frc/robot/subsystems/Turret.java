// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  public TalonFX turretMotor;
  // private TalonSRX elevator = new TalonSRX(Constants.ELEVATOR);
  Vision vision;
  double kp;
  double kf, ki;
  double maxIntegral;
  double deadZone;
  double fric;
  double posErr;
  double intErr;
  double zerr;
  double motorOutput;
  // private DigitalInput limitSwitch_center;
  double findTurretHomeSpeed = 0.3; // Thuis will be the speed to used for finding the turrets center position

  public boolean useEncoders;

  /** Creates a new Turret. */
  public Turret(Vision vision) {
    turretMotor = new TalonFX(Constants.TURRT_MOTOR_ID);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    turretMotor.setSelectedSensorPosition(0);
    turretMotor.setInverted(true);
    this.vision = vision;
    // limitSwitch_center = new DigitalInput(Constants.LIMIT_LEFT);

    kp = .15; // .2
    kf = 0*0.1;
    ki = 0*.0025;
    maxIntegral = 0*100;
    deadZone = 1;
    fric = 0;
    intErr = 0;
    posErr = 0;
    zerr = 0;

    resetEncoders();

    useEncoders = false;

  }

  public void resetEncoders(){
    turretMotor.setSelectedSensorPosition(0);
  }

  // public void findHub(double speed) {
  //   if (vision.getTarget() != 1) {
      
  //   }
  //   else {
  //     turretMotor.set(ControlMode.PercentOutput, 0);
  //     turnTurret(1);
  //   }
  // }
  public void setSpeed(double speed){
  
    // if(getLeftLimitSwitchStatus() && speed > 0){
    //   turretMotor.set(ControlMode.PercentOutput, -speed);
    //   Timer.delay(0.5);
    //   while (!getLeftLimitSwitchStatus()){}
    //   turretMotor.set(ControlMode.PercentOutput, speed);
    // }
  
    // Undo if set speed is broken
    if((limitSwitchEncoderLeft() && speed > 0) || (limitSwitchEncoderRight() && speed < 0) && useEncoders){
      turretMotor.set(ControlMode.PercentOutput, 0);
    }else{

      
        turretMotor.set(ControlMode.PercentOutput, speed);
      }
    // }
  }

  public double getEncoder(){
    return turretMotor.getSelectedSensorPosition();
  }
  // public void moveTurretPos(double position) {
  //   // 0 
  //   // position += 1000;
  //   // while (turretMotor.getSelectedSensorPosition() > position || (turretMotor.getSelectedSensorPosition() < position)) {
  //   //     if (turretMotor.getSelectedSensorPosition() > position) {
  //   //       setSpeed(-0.3);
  //   //     }
  //   //     else if (turretMotor.getSelectedSensorPosition() < position) {
  //   //       setSpeed(0.3);
  //   //     }
  //   //   }
  //   //   setSpeed(0);

  //     turretMotor.set(ControlMode.Position, position);
  // }

  // public void findTurretHome() {
  //   while (!getCenterLimitSwitchStatus()) {

  //     if (limitSwitchEncoderLeft()) {

  //       findTurretHomeSpeed *= -1;
      
  //     }
  //     else if (limitSwitchEncoderRight()) {

  //       findTurretHomeSpeed *= -1;
      
  //     }
  //     setSpeed(findTurretHomeSpeed);
      
  //   }
  //   setSpeed(0);
  //   resetEncoders();
  // }

  public boolean limitSwitchEncoderLeft() {
     if (turretMotor.getSelectedSensorPosition() >= 185000) {
       return true;
     } 
     return false;
  }
  public boolean limitSwitchEncoderRight() {
    if (turretMotor.getSelectedSensorPosition() <= -202298) {
      return true;
    } 
    return false;
 }
  public void turnTurret(double autoTrigger) {
    SmartDashboard.putNumber("VisTarget", vision.getTarget());

    SmartDashboard.updateValues();
    if(vision.getTarget() != 1.0){

      // // This will find the turret if in autonomous
      // if (limitSwitchEncoderLeft() || limitSwitchEncoderRight()) {

      //   findTurretHomeSpeed *= -1;
      
      // }
      // turretMotor.set(ControlMode.PercentOutput, findTurretHomeSpeed);

      // Undo if broken
      setSpeed(0);
      // turretMotor.set(ControlMode.PercentOutput, 0);
      return;
    }

    posErr = vision.getX();
    zerr = vision.getZ();

    intErr += posErr;

    fric = (kf / deadZone) * posErr;
    if (fric > kf) {
      fric = kf;
    }
    if (fric < -kf) {
      fric = -kf;
    }

    if (intErr > maxIntegral) {
      intErr = maxIntegral;
    }
    if (intErr < -1 * maxIntegral) {
      intErr = -1 * maxIntegral;
    }
    if(Math.abs(posErr) > .1 && posErr * intErr < 0){
      intErr = 0;
    }

    motorOutput = autoTrigger * (intErr * ki + posErr * kp + fric);
    // if (getLeftLimitSwitchStatus() == false && motorOutput < 0) {
    //   motorOutput = .25;
    // }
    // if (getRightLimitSwitchStatus() == false && motorOutput > 0) {
    //   motorOutput = -.25;
    // }
    setSpeed(-motorOutput);
    // turretMotor.set(ControlMode.PercentOutput, -motorOutput);

    // SmartDashboard.putNumber("Vision X: ", posErr);
    // SmartDashboard.putNumber("Vision Z: ", zerr);
    // SmartDashboard.putNumber("TARGET FOUND", vision.getTarget());
    // SmartDashboard.putNumber("VISION X", vision.getX());
    // SmartDashboard.putBoolean("LEFT LIMIT", getLeftLimitSwitchStatus());
    // SmartDashboard.putBoolean("RIGHT LIMIT", getRightLimitSwitchStatus());
    // SmartDashboard.putNumber("Turret Motor Ouptut: ", motorOutput);
  }

  // public boolean getCenterLimitSwitchStatus() {// right
  //   return !limitSwitch_center.get(); // Inverted
  // }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
