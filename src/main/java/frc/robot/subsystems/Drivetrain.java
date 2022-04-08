// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.ldap.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public WPI_TalonFX l1, l2, r1, r2;
  public MotorControllerGroup l, r;
  public DifferentialDrive ddrive;
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
  public DoubleSolenoid shifter;
  public Compressor c;

  // Auto
  
  public Encoder QuadEncoderL, QuadEncoderR;

  // public Solenoid shifterL, shifterR;
 // public AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
 // public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyroAngle)

  public Drivetrain() {
    // shifterL = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHIFTER_L);
    // shifterR = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHIFTER_R);
    l1 = new WPI_TalonFX(Constants.MOTOR_L1_ID);
    l2 = new WPI_TalonFX(Constants.MOTOR_L2_ID);
    r1 = new WPI_TalonFX(Constants.MOTOR_R1_ID);
    r2 = new WPI_TalonFX(Constants.MOTOR_R2_ID);
    c = new Compressor(0, PneumaticsModuleType.CTREPCM);
    r1.setInverted(true);
    r2.setInverted(true);
    l2.follow(l1);
    r2.follow(r1);

    c.enabled();
    c.enableDigital();

    // REMOVE
    // c.disable();


    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    l = new MotorControllerGroup(l1, l2);
    r = new MotorControllerGroup(r1, r2);
    // l.setInverted(true);
    // shifter.set(Value.kReverse);
 
    // Auto start
    QuadEncoderL = new Encoder(Constants.ENCODER_L_CHANNELA, Constants.ENCODER_L_CHANNELB);   
    QuadEncoderR = new Encoder(Constants.ENCODER_R_CHANNELA, Constants.ENCODER_R_CHANNELB); 
    
    QuadEncoderL.setReverseDirection(true);

    l1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    l1.setSelectedSensorPosition(0);
    r1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    r1.setSelectedSensorPosition(0);
    l2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    l2.setSelectedSensorPosition(0);
    r2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    r2.setSelectedSensorPosition(0);
    // Auto end

    // Sets it to low gear at the start if it isn't that already
    if (shifter.get() != DoubleSolenoid.Value.kForward) {
      shifter.set(DoubleSolenoid.Value.kForward);
    }

    // shifterL.set(false);
    // shifterR.set(false);
    // shifter.set(DoubleSolenoid.Value.kForward);
   
    // SmartDashboard.putBoolean("compressor enabled: ", pcmCompressor.enabled());
    ddrive = new DifferentialDrive(l, r);
    
  }

  public double getQuadSensor(){
    SmartDashboard.putNumber("ENCODERL_QUAD", QuadEncoderL.getDistance());
    SmartDashboard.updateValues();

    
    return QuadEncoderL.getDistance();
  }

  public void moveToBrake(){
    l1.setNeutralMode(NeutralMode.Brake);
    l2.setNeutralMode(NeutralMode.Brake);
    r1.setNeutralMode(NeutralMode.Brake);
    r2.setNeutralMode(NeutralMode.Brake);
  }

  public void moveToCoast(){
    l1.setNeutralMode(NeutralMode.Coast);
    l2.setNeutralMode(NeutralMode.Coast);
    r1.setNeutralMode(NeutralMode.Coast);
    r2.setNeutralMode(NeutralMode.Coast);
  }

  public double getLIntegratedSensor(){
    return l1.getSelectedSensorPosition();
  }

  public double getRIntegratedSensor(){
    return r1.getSelectedSensorPosition();
  }

  public void move(double power, double offset, boolean quickTurn){ // power is the throttle (drive stick), offset is turning
    // ddrive.arcadeDrive(power, offset);
    ddrive.curvatureDrive(power, offset, quickTurn);
    
  }

  public void setForward(){
    // shifterL.toggle();
    // shifterR.toggle();
    
    shifter.set(DoubleSolenoid.Value.kForward);
  }

  public void resetEncoders(){
    l1.setSelectedSensorPosition(0);
    r1.setSelectedSensorPosition(0);
    l2.setSelectedSensorPosition(0);
    r2.setSelectedSensorPosition(0);

    // l1.setNeutralMode(NeutralMode.Brake);
    // r1.setNeutralMode(NeutralMode.Brake);
    
    
    // QuadEncoderR.reset();
    // QuadEncoderL.reset();
    // QuadEncoderL.setDistancePerPulse(1.0/256.0);
    // QuadEncoderR.setDistancePerPulse(1.0/256.0);

    // l1.setInverted(InvertType.FollowMaster);
    // r1.setInverted(InvertType.FollowMaster);
  }

  // public void setEncoderDis(double v){
  //   QuadEncoderL.setDistancePerPulse(v);
  //   QuadEncoderR.setDistancePerPulse(v);
  // }

  public void setReverse(){
    // shifterL.toggle();
    // shifterR.toggle();
    shifter.set(DoubleSolenoid.Value.kReverse);
  }

  // public void moveAuto(double power, double distance){
  //   // 29273.5 encoder rotations per feet
  //   // double curangle = gyro.getDisplacementZ();
  //   // double kp = 0.01;
  //   // double offset = kp * curangle;
  //   while (l1.getSelectedSensorPosition() <= (distance*29273.5)) {
  //     move(power, 0);
  //   }
  //   // Timer.delay(time);
  //   move(0, 0);
  //   // gyro.getDisplacementZ()
  //   SmartDashboard.putNumber("Displaement x(ft)", l1.getSelectedSensorPosition()/29273.5);

  //   SmartDashboard.updateValues();
  // }
  // public void turnDegrees(double degrees) {
  //   SmartDashboard.putNumber("GYROYAW", gyro.getYaw());
  //   SmartDashboard.updateValues();

  //   if (degrees < 0) {// degrees = -90
  //     if (gyro.getYaw() >= degrees) {
 
  //       move(0, 0.3);
  //     }
  //   }
  //   else { // Degrees = 90
  //     if (gyro.getYaw() <= degrees) {
       
  //       move(0, -0.30);
  //     }
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
