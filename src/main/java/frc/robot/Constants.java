/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DrivetrainConstants {
        public static boolean isTankDriveSquared = false;
        public static boolean doesCheesyDrivePivot = false;
        public static boolean isArcadeDriveSquared = true;

        public static double rotationDeadband = 0.70;
        public static double speedDeadband = 0.30;
    }

    public static class MecanumDrivetrianConstants {
        //Mecanum ID names
        public static int leftFrontTalonID = SpeedControllerCanPortConstants.driveFrontLeftTalon;
        public static int leftRearVictorID = SpeedControllerCanPortConstants.driveRearLeftVictor;
        
        public static int rightFrontTalonID = SpeedControllerCanPortConstants.driveFrontRightTalon;
        public static int rightRearVictorID = SpeedControllerCanPortConstants.driveRearRightVictor;

        public static double rotationDeadband = .25 ;
        public static double strafingDeadband = .25;  //.75
        public static double speedDeadband = 0.25;
        public static double m_maxOutput = .75;
        public static boolean isTankDriveSquared = false;
        public static boolean doesCheesyDrivePivot = false;
        public static boolean isArcadeDriveSquared = true;

    }

    public static class SpeedControllerCanPortConstants {
        //mecanum drive motor ID ports
        //left side
        private static int driveFrontLeftTalon = 7;
        private static int driveRearLeftVictor = 9;

        //right side
        private static int driveFrontRightTalon = 1;
        private static int driveRearRightVictor = 3;
    }

    public static final int CONTROLLER_PORT = 1;

    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int RIGHT_STICK_X = 4;
    public static final int RIGHT_STICK_Y = 5;
}
