/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.Turret;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int leftMaster = 20;
    public static final int leftSlave = 21;
    public static final int rightMaster = 22;
    public static final int rightSlave = 23;

    public static final int shooterL = 26;
    public static final int shooterR = 27;
    public static final int turretMotor = 28;
    public static final int kickerMotor = 29; 

    public static final int leftjoystick = 24;
    public static final int rightjoystick = 25;

    public static final int indexerMotor = 30;
    public static final int intakeSensor = 31;
    public static final  int indexerTopSensor = 32;
    public static final int indexerBottomSensor = 33;
    public static int intakeMotor;
    public static int pcmOne;
    public static int intakePistonForward;
    public static int intakePistonReverse;
}
