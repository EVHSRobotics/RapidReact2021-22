// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.testSystem;

public class testcommand extends CommandBase {
  private TalonFX testtalon1;
  private testSystem tester;
  private final Joystick stick;

  /** Creates a new testcommand. */
  public testcommand(Joystick stick, testSystem tester) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stick = stick;
    this.tester = tester;
    addRequirements(tester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tester.testMethod(8000);
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
