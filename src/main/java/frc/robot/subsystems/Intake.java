package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public TalonFX conveyorMotor;
  public VictorSPX brushMotor;
  public TalonSRX transitionMotor;
  // private Compressor compressor;

  private DigitalInput banner1; // dio 0
  private DigitalInput banner2; // dio 1
  // private DigitalInput banner3; // dio 2

  private Spark ledDigital;

  private boolean storageFull; // whether storage is full
  private int ballStored; // balls stored
  private String intakeStatus;
  private DoubleSolenoid leftSolenoid, rightSolenoid; 

  private boolean intakeDown;
  private double joyIntakeSpeed = 0, conIntakeSpeed = 0;
  
  public Intake() {
    // compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    
    conveyorMotor = new TalonFX(Constants.INTAKE_MOTOR);
    brushMotor = new VictorSPX(Constants.BRUSH_MOTOR);
    transitionMotor = new TalonSRX(Constants.TRANSIT_MOTOR);
    conveyorMotor.setInverted(true);
    transitionMotor.setInverted(true);

    // compressor.enableDigital();
    banner1 = new DigitalInput(Constants.BANNER_1);
    banner2 = new DigitalInput(Constants.BANNER_2);

    ledDigital = new Spark(Constants.LED_CHANNEL);

    // banner3 = new DigitalInput(Constants.BANNER_3);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKEPISTON1_CHANNEL1, Constants.INTAKEPISTON1_CHANNEL2);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKEPISTON2_CHANNEL1, Constants.INTAKEPISTON2_CHANNEL2);
    
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);

    transitionMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
 
  // public void intake(boolean isEjecting){
  //   if (!isEjecting) {

  //     if (!banner1Output()) {
  //       storageFull = true; // one slot open
  //       ballStored = 2;
  //     } else if (!banner2Output()) {
  //       storageFull = false; // two slots open 
  //       ballStored = 1;
  //     } else {
  //       storageFull = false;
  //       ballStored = 0;
  //     }

  //     if (storageFull) {
  //       conveyor(0);
  //       intakeStatus = "intake motor not running";
  //     } else {
  //       conveyor(.5);
  //       intakeStatus = "intake motor running";
  //      }
  //   }else{
  //     conveyor(-1);
  //     transitionMotor(-1); // ejects everything 
  //     intakeStatus = "intake motor ejecting balls";
  //   } 

  //   SmartDashboard.putBoolean("Storage Full?", storageFull);
  //   SmartDashboard.putNumber("Balls Stored:", ballStored);
  //   SmartDashboard.putString("Intake Status:", intakeStatus);

  // }

  public void mainIntakeFunction(double speed){
    // SmartDashboard.putBoolean("BANNER 1", banner1Output());
    // SmartDashboard.putBoolean("BANNER 2", banner2Output());
    // SmartDashboard.updateValues();
    // if(banner1Output() && banner2Output()){
    //   //stop the conveyer
    //   conveyor(0);
    //   intakeBrush(0);
    // }
    // else{
      intakeBrush(speed);
      conveyor(speed);
    // }
    
  }

  public void setLEDDigital(double ledConstant) {
    ledDigital.set(ledConstant);
  }
  public void IntakeBalls(double speed){
    intakeBrush(speed);
    //don't use the conveyer keep the balls below
    conveyor(speed);
  }

  public void ShootBalls(double speed){
    conveyor(speed);
    transitionMotor(speed);
  }

  public void conveyor(double speed){
    // if ((!banner1Output()) && (!banner2Output())) {
      
      
    // } else {
      conveyorMotor.set(ControlMode.PercentOutput, speed); 
      // intakeBrush(speed);
    // }
  }

  public void intakeBrush(double speed) {
    if (speed > 0.85) {
      speed = 0.85;
    }

    brushMotor.set(ControlMode.PercentOutput, speed);

  }

  public void transitionMotor(double speed) {
    transitionMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean banner1Output() { // banner sensor for 1st ball
    return banner1.get();
  }
  public boolean banner2Output() { // banner sensor for 2nd ball
    return banner2.get();
  }
  // public boolean banner3Output() { // banner sensor for transition
  //   return banner3.get();
  // }

  // public void ejectBalls(double speed) {
  //   intakeMotor.set(ControlMode.PercentOutput, speed);

  // }

  public void intakeToggle(){
    leftSolenoid.toggle();
    rightSolenoid.toggle();
    if ((leftSolenoid.get() == Value.kForward) && (rightSolenoid.get() == Value.kForward)){
      intakeDown = true;
      // conveyor(0);
    } else {
      intakeDown = false;
      intakeBrush(0);
      // conveyor(0);
    }


  }

  public void intakeUp() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }
  public void intakeDown() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }
  public boolean getIntakeDown() {
    return intakeDown;
  }

}
