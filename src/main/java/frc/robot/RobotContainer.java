/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Turret m_Turret = new Turret();

  private final Shooter m_Shooter = new Shooter();

  private final Kicker m_Kicker = new Kicker();

  private final Joystick leftJoystick = new Joystick(Constants.leftjoystick);

  private final Joystick rightJoystick = new Joystick(Constants.rightjoystick);

  private final JoystickButton oneButton = new JoystickButton(leftJoystick, 1);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    oneButton.whileHeld(new Kick(m_Kicker));
  }
  private double leftJoystickX() {
    return leftJoystick.getX();
  }
  private double rightJoystickY() {
    return rightJoystick.getY();
  }
  private Shoot shoot = new Shoot(m_Shooter, rightJoystickY());

  private Rotate rotate = new Rotate(m_Turret, leftJoystickX());
  
  m_Shooter.setDefaultCommand(shoot);
  m_Turret.setDefaultCommand(rotate);
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
