// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutoCommand extends CommandBase {
  /** Creates a new AutoCommand. */
  Drivetrain drivetrain;
  Turret turret;
  Vision vision;
  Shooter shooter;
  Intake intake;
  boolean run = false;
  // Auto 1 = 5 ft, Auto Aim
  // Auto 2 = 5 ft, without Auto Aim
  // Auto 3 = 7 ft, Auto Aim
  boolean far = false; // far = true : 7 ft && far = false = 5 ft
  boolean autoAim = true;
  boolean autoCompute = true;

  int autoCommandParam = 3;

  public AutoCommand(Drivetrain drivetrain, Turret turret, Vision vision, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.vision = vision;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(drivetrain);

   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    run = false;
    drivetrain.resetEncoders();
    // drivetrain.gyro.resetDisplacement();
    // drivetrain.gyro.reset();
    // drivetrain.setEncoderDis(1.0/256.0);
    // drivetrain.l.setInverted(true);

     // THis runs the right auto command param
     if (autoCommandParam == 1) {
      far = false;
      autoAim = false;
      autoCompute = false;
    }
    else if (autoCommandParam == 2) {
      far = false;
      autoAim = true;
      autoCompute = false;

    }
    else if (autoCommandParam == 3) {
      far = true;
      autoAim = false;
      autoCompute = false;
    }
    else if (autoCommandParam == 4) {
      far = true;
      autoAim = true;
      autoCompute = false;
    }
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    // drivetrain.moveAuto(0.4, 4.5); // 6 ft
    // drivetrain.turnDegrees(90);

    Timer.delay(2);
    if(!run){
      phase2();
      run = true;
    }
    // if (((startTime - System.currentTimeMillis()) / 1000.0) <= 15) {
    //   Timer.delay(((startTime - System.currentTimeMillis()) / 1000.0));
    // }
    // Shoots
    // phase1();
    // // Drives backward
    // phase2();
    // // Shoots
    // phase1();
    
    // SmartDashboard.putNumber("L Encoder:", drivetrain.getIntegratedSensor());
    // SmartDashboard.updateValues();
    // intake.conveyor(0.5);
    // while (vision.getTarget() != 1.0){}
    // while (Math.abs(vision.getX()) < 0.02){
    //   turret.turnTurret(1);
    // }
    // double rpm = shooter.computeV(vision.getY());
    // shooter.outakeV(rpm);
    // while(Math.abs(shooter.getRPM() - rpm) <= 100){}
    // intake.transitionMotor(1);
  }

  public void phase1() { // shoot with auto aim


    // Checks the alliance color
    // if (shooter.teamColor.equals(DriverStation.Alliance.Red) && colorReading.red > colorReading.blue) {
    //   // Calculates RPM
     //  double rpm = shooter.computeV(vision.getY());

    //   // Starts Revving up motor
    //   shooter.outakeV(rpm);   
    // }
    // else if (shooter.teamColor.equals(DriverStation.Alliance.Blue) && colorReading.red < colorReading.blue) {
    //   // Calculates RPM

        
    // }
    // else {
    //     shooter.outtakeBall(0.5);
    // }
      Timer.delay(1);
    double startTime = System.currentTimeMillis();

    // Locks turret on target
    // turret.setSpeed(0);
    // turret.turnTurret(0.5);
    // Locks turret on target
   
  

  

    SmartDashboard.putNumber("UPDATED AUTO VISION", vision.getY());
    SmartDashboard.updateValues();
    if (autoCompute) {
      double rpm = shooter.computeV(vision.getY()) + 1900.0;

      // Starts Revving up motor
      shooter.outakeV(rpm);
    }
    else {
      if (far) {
        shooter.outakeV(17000);
      }
      else {
        shooter.outakeV(16000);
      }
    }
    
    
    // Makes sure RPM meets minimum requirement
    // while (shooter.getRPM() < rpm) {}
    Timer.delay(3);
    
    // While there is still a ball loaded start transition(aka shoot)
    intake.conveyor(1);
    intake.transitionMotor(1);
    Timer.delay(3);
    intake.conveyor(0);
    intake.transitionMotor(0);

    // Stops shooter motor
    shooter.outtakeBall(0);
  }

  public void phase2() {
    intake.intakeDown(); //drops intake
    Timer.delay(1);
    // Gets the current start time
    intake.transitionMotor(-1);
//
    shooter.outakeV(10000);
    //rev up shooter
    double startTime = System.currentTimeMillis();
    // 6 second limit or banner 2 ouput become true will make this stop
    // && (!intake.banner1Output() || !intake.banner2Output())
    double distance = 3.25; // Set to far by default
    if (!far) {
      distance = 2.75;
    }
    
    while ((((System.currentTimeMillis() - startTime) / 1000.0) <= distance)) {
      SmartDashboard.putNumber("CURRENT TIME:", System.currentTimeMillis() - startTime);
      SmartDashboard.updateValues();
      // Run conveyor and move backward
      intake.intakeBrush(1);
      intake.conveyor(1);
      drivetrain.move(0.4, 0, false);

      if (autoAim) {
        // while ((((System.currentTimeMillis() - startTime) / 1000.0) <= 2.0)) {
          turret.turnTurret(1);
        // }
        // // This stops the turning
      // turret.setSpeed(0);   
      }
    }
    SmartDashboard.putBoolean("REACHED", true);
    SmartDashboard.updateValues();
    // Stops drivetrain(safety)
    // intake.intakeBrush(0);
    // intake.conveyor(0);
    turret.setSpeed(0);   
    drivetrain.move(0, 0, false);
    Timer.delay(0.3);
    intake.intakeBrush(0);
    intake.conveyor(0);
    // Pulls back up the intake
    intake.intakeToggle();

    phase1();

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
