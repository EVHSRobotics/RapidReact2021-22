/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // private static TalonFX intakeMotor1 = new TalonFX(Constants.INTAKE_1);
  // private static TalonFX intakeMotor2 = new TalonFX(Constants.INTAKE_2);
  // private static TalonFX turretMotor = new TalonFX(Constants.TURRET);
  public WPI_TalonFX shooterMotor1;
  public WPI_TalonFX shooterMotor2;
  //private static VictorSPX intakeMotor = new VictorSPX(Constants.INTAKE);
  // private DigitalInput banner;
  boolean ballLoaded;
  // private DigitalInput banner2;

  private ColorSensorV3 shooterColorSensor;

  private double currentSpeed;
  private double maxSpeed;
  
  // This is the color of the balls that our team can shoot out properly
  // Either RED or BLUE
  public DriverStation.Alliance teamColor = DriverStation.getAlliance();

  Timer timer;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterMotor1 = new WPI_TalonFX(Constants.SHOOTER1);
    shooterMotor2 = new WPI_TalonFX(Constants.SHOOTER2);
    shooterMotor1.setInverted(true);
    shooterMotor2.follow(shooterMotor1);
    // banner = new DigitalInput(Constants.BANNER_1);
    // banner2 = new DigitalInput(8);
    ballLoaded = false;
    timer = new Timer();
    // shooterMotor1.setNeutralMode(NeutralMode.Coast);
    // shooterMotor2.setNeutralMode(NeutralMode.Coast);

    shooterColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Color getColorSensorV3() {
    return shooterColorSensor.getColor();
  }

  public double getShooterVel() {
  
    return (shooterMotor1.getSelectedSensorVelocity() + shooterMotor2.getSelectedSensorVelocity()) / 2;
  }

  public void outtakeBall(double speed) {
    // maxSpeed = SmartDashboard.getNumber("SHOOTER SPEED: ", 0);
    shooterMotor1.set(ControlMode.PercentOutput, speed);
    // shooterMotor1.set(ControlMode.PercentOutput, speed);
// 
    // if (speed > 0.8) {
    //   shooterMotor1.set(ControlMode.PercentOutput, 0.8);

    // }
    // else {
    //   shooterMotor1.set(ControlMode.PercentOutput, speed);

    // }
    // getShooterVel();
    // SmartDashboard.putNumber("SHOOTER SPEED: ", speed*18000);
      
    // } catch (Exception e) {
    //   //TODO: handle exception
    // }
    // if(speed > 0.1) shooterMotor1.set(ControlMode.PercentOutput, speed);
    // else shooterMotor1.set(ControlMode.Velocity, 0);
    // // shooterMotor1.set(ControlMode.Velocity, );
    // // if(speed > 0.5) shooterMotor1.set(ControlMode.Velocity, 15000);
    // // else shooterMotor1.set(ControlMode.Velocity, 0);
    // SmartDashboard.putNumber("Shooter speed 1 : ", shooterMotor1.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter speed 2 : ", shooterMotor2.getSelectedSensorVelocity());
  }

  public double computeV(double ty){
    // Current uncommented is 8.5 ft
    return 16054 - 398 * ty + 10.2 * ty * ty - 100;
    // return (16297 - 425 * ty + 24.6 * ty * ty + 75) ;
  }

  public void outakeV(double v){
    // SmartDashboard.putNumber("Kshitij", 0);
    shooterMotor1.set(ControlMode.Velocity, v);
  }

  public void inttakeBall(double speed) {
    
    //intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  // public boolean bannerOutput() {
  //   return banner.get();

  // }

  // public boolean banner2Output() {
  //   return banner2.get();
  // }

  public void ejectBalls(double speed) {
    //intakeMotor.set(ControlMode.PercentOutput, speed);

  }

  public void intake(boolean isEjecting) {
   // System.out.println("Is ejecting: " + isEjecting);
    // if (!isEjecting) {
    //   if (bannerOutput()) {
    //     ballLoaded = true;
    //     timer.start();
    //   } else if (banner2Output()) {
    //     ballLoaded = false;
    //   }
    //   if(timer.get() > .5){
    //     ballLoaded = false;
    //   }
    //   if (ballLoaded) {
    //     inttakeBall(0.5);
    //   } else {
    //     inttakeBall(0);
    //   }
    // }else{
    //   inttakeBall(-1);
    // }
  }

  public double getRPM(){
    return (shooterMotor1.getSelectedSensorVelocity() + shooterMotor2.getSelectedSensorVelocity()) / 2;
  }

}
