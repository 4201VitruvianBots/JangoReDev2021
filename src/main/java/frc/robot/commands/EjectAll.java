/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class EjectAll extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private final Intake m_intake;

    /**
     * Creates a new ExampleCommand.
     *
     * @param indexer a subsystem used by this command.
     * @param intake  a subsystem used by this command.
     */

    public EjectAll(Indexer indexer, Intake intake) {
        m_indexer = indexer;
        m_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerOutput(-0.6);
        m_indexer.setKickerOutput(-0.5);
        m_intake.setIntakePercentOutput(-0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_indexer.setKickerOutput(0);
        m_indexer.setIndexerOutput(0);
        m_intake.setIntakePercentOutput(0);
        m_intake.updateCounter(false, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}