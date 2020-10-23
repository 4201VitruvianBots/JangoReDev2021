/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;
import frc.robot.Constants;
/**
 * An example command that uses an example subsystem.
 */
public class DriveSensorToPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double m_power, m_xPosition, m_yPosition; 
  private double distanceWithinPosition = 0.5; // How close robot has to be to target position for command to finish

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveSensorToPosition(DriveTrain drivetrain, double power, double xPosition, double yPosition) {
    m_drivetrain = drivetrain;
    m_power = power;
    m_xPosition = xPosition;
    m_yPosition = yPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setTankDrive(m_power, m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPosition = m_drivetrain.getRobotPosition();
    double distance = Math.sqrt(Math.pow(currentPosition.getTranslation().getX() - m_xPosition, 2) + 
    Math.pow(currentPosition.getTranslation().getY() - m_yPosition, 2));
    return distance <= distanceWithinPosition;
  }
}
