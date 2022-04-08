// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretMove extends CommandBase {
  /** Creates a new TurretMove. */
  Turret turret;
  XboxController controller;
  boolean manual;
  Vision vision;
  boolean limR;
  boolean limL;

  private NetworkTableEntry turretAutoEntry;


  public TurretMove(Turret turret, XboxController controller, Vision vision) {
    this.turret = turret;
    addRequirements(turret);
    this.vision = vision;
    this.controller = controller;

    turretAutoEntry = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Turret Auto", true).getEntry();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manual = true; // Starts in manual mode
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("TURRETTHROTTLE", turretThrottle);
    // turret.turnTurret(-0.1);
    /*
    if (wheel.getRawButton(4)){
      turret.turnTurret(-0.5); // turns turret left
    } else if (wheel.getRawButton(6)){
      turret.turnTurret(0.5); // turns turret right
    }
    */
    // SmartDashboard.putNumber("AXIS VALUE", controller.getRightTriggerAxis());
    // SmartDashboard.updateValues();
    SmartDashboard.putNumber("Turret Controller Left X", controller.getLeftX());
    SmartDashboard.putBoolean("MANUAL ", manual);
    SmartDashboard.updateValues();
    if(controller.getBButtonPressed()){

      // while (vision.getTarget() == 1 && vision.getX() > 0.1 && vision.getX() < -0.1) {
      //   turret.turnTurret(1);
      // }

     
      manual = false;
    }
    // else if(controller.getBButtonReleased()){
    //   manual = true;
    //   while (turret.turretMotor.getSelectedSensorPosition() > 1000 || (turret.turretMotor.getSelectedSensorPosition() < 1000)) {
    //     if (turret.turretMotor.getSelectedSensorPosition() > 10000) {
    //       turret.setSpeed(-0.3);
    //     }
    //     else if (turret.turretMotor.getSelectedSensorPosition() < 10000) {
    //       turret.setSpeed(0.3);
    //     }
    //   }
    //   turret.setSpeed(0);
    // }

      if ((controller.getLeftX() > 0.1) || (controller.getLeftX() < -0.1)){
      //   turret.setSpeed(0);
      // } else {
        manual = true;

        turret.setSpeed(-controller.getLeftX());
        // manual = true;
      }
      else {
        turret.setSpeed(0);
      }
     
      // else {
      //   turret.setSpeed(0);
      // }

      
      SmartDashboard.putNumber("POV", controller.getPOV(0));
      SmartDashboard.updateValues();
      if (controller.getPOV(0) != -1) {
        turret.useEncoders = !turret.useEncoders;

        if (turret.useEncoders) {
          turret.resetEncoders();

        }
      
      }

      if (turret.useEncoders) {
        controller.setRumble(RumbleType.kLeftRumble, 0);

      }
      else {
        controller.setRumble(RumbleType.kLeftRumble, 0.3);
      }

      // else {
      //   turret.setSpeed(0);
      // }
      if(!manual){
        // if ((controller.getLeftX() < 0.2) && (controller.getLeftX() > -0.2)){
      //   turret.turnTurret(0);
      // } else {
      //   // if (vision.getX() < 1.2) {
      //   //   turret.turnTurret(-controller.getLeftX());
      //   // } else {
      //     turret.turnTurret(controller.getLeftX());
      //   // }
      // }
      turret.turnTurret(1);
      // manual = true;
    }

    
    SmartDashboard.putNumber("Turret Sensor Pos", turret.turretMotor.getSelectedSensorPosition());
    SmartDashboard.updateValues();
    // This will move the turret until it finds the hub oscillating between the limit switches
    // turret.findHub(0.4);

    // turret.setSpeed(controller.getRightX());

    // if (turret.getLeftLimitSwitchStatus()) {
    //   limL = true;
    // } else {
    //   limL = false;
    // }

    // if (turret.getRightLimitSwitchStatus()) {
    //   limR = true;
    // } else {
    //   limR = false;
    // }
    // limL = turret.getLeftLimitSwitchStatus();
    // limR = turret.getRightLimitSwitchStatus();

    SmartDashboard.putBoolean("Limit Right", limR);
    SmartDashboard.putBoolean("Limit Left", limL);

    if (!manual) {
      turretAutoEntry.setBoolean(true);
    }
    else {
      turretAutoEntry.setBoolean(false);
    }

    
    Shuffleboard.update();
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

