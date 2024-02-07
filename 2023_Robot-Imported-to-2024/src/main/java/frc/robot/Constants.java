// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {

        public static final int timeoutMS = 30;

        // 30AMP CURRENT LIMITS
        public static final int current30AmpPeakCurrentLimit = 25;
        public static final int current30AmpPeakCurrentDuration = 150;
        public static final int current30AmpContinuousCurrentLimit = 25;

        // 40AMP CURRENT LIMITS
        public static final int current40AmpPeakCurrentLimit = 35;
        public static final int current40AmpPeakCurrentDuration = 150;
        public static final int current40AmpContinuousCurrentLimit = 35;

        //Drive Motor Ports
        public static final int leftMasterPort = 2;
        public static final int leftSlavePort = 3;
        public static final int rightMasterPort = 0;
        public static final int rightSlavePort = 1;

        //Drive Joystick Values
        public static final int leftThumbstickY = 0;
        public static final int leftThumbstickX = 1;

        
        //TUrn PID Values
        public static final double kTurnP = -.03;
        public static final double kTurnI = 40;
        public static final double kTurnD = 0;
        public static final boolean kGyroReversed = false;
        public static final double kMaxTurnRateDegPerS = 20;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;
        //Angle Tolerance
        public static final double kTurnToleranceDeg = 7;
        //Angular Rate
        public static final double kTurnRateToleranceDegPerS = 20;
    }

    public static final class CameraConstants{
        public static final int camera1 = 0;
        public static final int camera2 = 1;
    }

    public static final class IntakeConstants {
        public static final int armRaisePort = 0;
        public static final int armExtendPort = 1;

        public static final int retractLimitSwitch = 3;
        public static final int upperLiftLimitSwitch = 7;
        public static final double armExtendSpeed = .82; //Extend Arm
        public static final double armRetractSpeed = -.7; //Retract arm
        public static final double armRaiseSpeed = .5; //raise arm
        public static final double armLowerSpeed = -.1
        ; //lower arm
        public static final double liftedMotorLimit = .18; //If limit switch added
        public static final double liftMotorMinimumLimit = .22; //Hold Bucket up
        public static final double slowArmRaiseSpeed = 0.15; //raise arm slowly
        public static final double slowArmLowerSpeed = -.02; //lower arm slowly

    }



}
