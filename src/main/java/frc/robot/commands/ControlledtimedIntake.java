/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Enums.IntakeStates;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ControlledtimedIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private final Intake m_intake;

    private final double intakeRPM = 5000;
    private final double indexRPM = 300;
    private double timestamp, intakeTimestamp, indexerTimestamp, fourBallTimestamp;
    private boolean intaking, haveFour, haveFourTripped;
    private final Joystick m_controller;

    private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;

    /**
     * Creates a new ControlledtimedIntake.
     */
    public ControlledtimedIntake(Intake intake, Indexer indexer, Joystick controller) {
        m_intake = intake;
        m_indexer = indexer;
        m_controller = controller;
        m_time = 10;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setIntakingState(true);
        timestamp = Timer.getFPGATimestamp();
        if (m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
            intakeState = IntakeStates.INTAKE_FIVE_BALLS;
        else if (m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
            intakeState = IntakeStates.INTAKE_FOUR_BALLS;
        else
            intakeState = IntakeStates.INTAKE_ONE_BALL;


        startTime = Timer.getFPGATimestamp();
        if (m_intake.getIntakePistonExtendStatus() != true)
            m_intake.setintakePiston(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (intakeState) {
            case INTAKE_FIVE_BALLS:
                m_intake.setIntakePercentOutput(0);
                m_indexer.setKickerOutput(0);
                m_indexer.setIndexerOutput(0);
                m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                break;
            case INTAKE_FOUR_BALLS:
                m_intake.setIntakePercentOutput(0.8);
                m_indexer.setKickerOutput(0);
                if (m_indexer.getIntakeSensor())
                    intakeState = IntakeStates.INTAKE_FIVE_BALLS;
                break;
            case INTAKE_ONE_BALL:
            default:
                m_intake.setIntakePercentOutput(0.8);
                m_indexer.setKickerOutput(-0.4);
                if (m_indexer.getIndexerBottomSensor()) {
                    m_indexer.setIndexerOutput(0.95);
                } else {
                    m_indexer.setIndexerOutput(0);

                }

                // Called once the command ends or is interrupted.
                @Override
                public void end ( boolean interrupted){
                m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                m_intake.setIntakingState(false);
                m_intake.setIntakePercentOutput(0);
                m_indexer.setIndexerOutput(0);
                m_indexer.setKickerOutput(0);
                if (intakeState == IntakeStates.INTAKE_FIVE_BALLS)
                    m_intake.setintakePiston(false);
            }

            // Returns true when the command should end.
            @Override
            public boolean isFinished () {
                return Timer.getFPGATimestamp() - startTime > m_time;
                return intakeState == INTAKE_FIVE_BALLS
            }
        }
