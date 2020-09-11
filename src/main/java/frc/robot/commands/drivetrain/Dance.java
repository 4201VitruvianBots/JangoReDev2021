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
public class Dance extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Dance(DriveTrain drivetrain) {
    addCommands(
      new DriveForward(drivetrain, 2, 1),
      new DriveBackward(drivetrain, 2, 1),
      new PointTurn(drivetrain, true, 10, 1),
      new DriveForward(drivetrain, 2, 1),
      new DriveBackward(drivetrain, 2, 1),
      new PointTurn(drivetrain, true, 10, 1)
    );
  }

}
