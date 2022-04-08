// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.JoyDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurretMove;
import frc.robot.commands.testcommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.commands.testcommand;
import frc.robot.subsystems.testSystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Vision vision = new Vision();
  public final Turret turret = new Turret(vision);
  private final testSystem tSys = new testSystem();
  private final Climber climber = new Climber();
  private Joystick driveStick, turnStick;
  private XboxController controller, driveController;
  private final JoyDrive jdrive;
  private final ClimberCommand climb;
  private final TurretMove tmove;
  private final Shooter shooter = new Shooter();
  private final Shoot shoot;
  // private final MoveClimb climbmove;
  private final Intake intake = new Intake();
  private final IntakeBall intakeCommand;
  private final testcommand tester;
  private final AutoCommand autoCommand;
  private boolean config = false;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveStick = new Joystick(Constants.DRIVE_STICK_PORT);
    turnStick = new Joystick(Constants.TURN_STICK_PORT);
    controller = new XboxController(Constants.XBOX_PORT);
    driveController = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    tmove = new TurretMove(turret, controller, vision);
    intakeCommand = new IntakeBall(intake, controller, driveStick);
    autoCommand = new AutoCommand(drivetrain, turret, vision, shooter, intake);
    climb = new ClimberCommand(climber, driveController);
    // adds intake for the auto to know when 
    jdrive = new JoyDrive(drivetrain, driveStick, turnStick, intakeCommand, driveController);
    shoot = new Shoot(shooter, vision, driveStick, intake, controller);
    tester = new testcommand(driveStick, tSys);

    configureTalons();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command[] getTeleCommand() {
    // An ExampleCommand will run in autonomous
    Command[] ret = {jdrive, intakeCommand, tmove, shoot, climb};
    return ret;
    // Removed tmove from ret
    /*
    REMINDER: schedule the other commands here !!!!
    */
  }

  public Command getAutoCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
    // Removed tmove from ret
    /*
    REMINDER: schedule the other commands here !!!!
    */
  }

  // Auto Commands only once
  public void startAutoInit() {

    // Init Commands
    // drivetrain.encoderL.reset();
    // drivetrain.encoderR.reset();

    // // In 4 feet
    // drivetrain.encoderL.setDistancePerPulse(1.0/256.0);
    // drivetrain.encoderR.setDistancePerPulse(1.0/256.0);
    
  }

  public void startAutoPeriod() {
// moves robot 4 feet backwards

    // if (drivetrain.encoderL.getDistance() <= 4) {

    //   drivetrain.ddrive.arcadeDrive(0.5, 0);
      

    // }
    // else {
    //   // Turns turret to left if at right and right if at left and if it is between
    //   // -1 and 1 it just disables the turret and starts shooting
    //     if (vision.getX() > 1.0) {
    //       turret.turnTurret(-1);
    //     }
    //     else if (vision.getX() < -1.0) {
    //       turret.turnTurret(1);

    //     }
    //     else {
    //       turret.turnTurret(0);
        
          
    //       // At this point in the auto the robot should have the ball in its intake but facing in the wrong direction
    //       // double intakeSpeedRev = shoot.computeV(vision.getY());
          

    //       // Until the intake is empty keep shooting
    //       if (intake.banner1Output()) {
    //         // shooter.outakeV(intakeSpeedRev);
    //         intake.conveyor(0.5);
    //         // Waits until the shooters velocity is within 50 rpm of the vision based speed
    //         // if (shooter.getShooterVel() > (intakeSpeedRev - 50)) {
    //           // intake.transitionMotor(1);
    //         // }
    //       }
    //     }
    // }
  }

  public void configureTalons() {
    drivetrain.l1.setNeutralMode(NeutralMode.Brake);
    drivetrain.l2.setNeutralMode(NeutralMode.Brake);
    drivetrain.r1.setNeutralMode(NeutralMode.Brake);
    drivetrain.r2.setNeutralMode(NeutralMode.Brake);
    
    intake.brushMotor.setNeutralMode(NeutralMode.Coast);
    intake.conveyorMotor.setNeutralMode(NeutralMode.Brake);
    intake.transitionMotor.setNeutralMode(NeutralMode.Brake);

    shooter.shooterMotor1.setNeutralMode(NeutralMode.Coast);
    shooter.shooterMotor2.setNeutralMode(NeutralMode.Coast);

    turret.turretMotor.setNeutralMode(NeutralMode.Brake);
    
    config = true;
    
    SmartDashboard.putBoolean("Talons Configured", config);
    SmartDashboard.updateValues();

  }

  // public void intakeUp() {
  //   intake.intakeUp();
  // }
}
