// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends CommandBase {

  Climber climberSys;



  XboxController controller;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(Climber climberSys, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.climberSys = climberSys;
    this.controller = controller;


    addRequirements(climberSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSys.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(controller.getBButtonPressed()){
    //   climberSys.resetEncoders();
    // }
    
    SmartDashboard.putNumber("MOTOR 7 ENCODER: ", climberSys.get7());
    SmartDashboard.putNumber("MOTOR 8 ENCODER: ", climberSys.get8());
    SmartDashboard.updateValues();

    // if (controller) {
    //   climberSys.move(0.3);
    // }
    // else if (controller.getPOV(4) != -1) {
    //   climberSys.move(-0.3);
    // }
    // else {
    //   climberSys.move(0);
    // }

    if(controller.getRightTriggerAxis() > 0.1 && climberSys.get7() < 244348){
      climberSys.move(controller.getRightTriggerAxis() * 0.65);
      // climberSys.get8() > 0
    }else if(Math.abs(controller.getLeftTriggerAxis()) > 0.1 && climberSys.get7() > 0){
      climberSys.move(-controller.getLeftTriggerAxis() * 0.65);
    }else{
      climberSys.move(0);
    }
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
