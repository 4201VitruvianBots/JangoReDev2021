/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class JFuMaze extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JFuMaze(DriveTrain drivetrain) {
    addCommands(
      new DriveForwardDistance(drivetrain, 96, 0.5),
      new PointTurnAngle(drivetrain, 90, 0.5),
      new DriveForwardDistance(drivetrain, 24, 0.5),
      new PointTurnAngle(drivetrain, 90, 0.5),
      new DriveForwardDistance(drivetrain, 72, 0.5),
      new PointTurnAngle(drivetrain, -90, 0.5),
      new DriveForwardDistance(drivetrain, 48, 0.5),
      new PointTurnAngle(drivetrain, -90, 0.5),
      new DriveForwardDistance(drivetrain, 48, 0.5),
      new PointTurnAngle(drivetrain, 90, 0.5),
      new DriveForwardDistance(drivetrain, 24, 0.5),
      new PointTurnAngle(drivetrain, -90, 0.5),
      new DriveForwardDistance(drivetrain, 48, 0.5)
    );
  }

}
