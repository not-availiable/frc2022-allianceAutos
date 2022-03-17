// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

  // other constants

  public static final class AutoConstants {
    public static final int kFrontLeftFirstEncoderPort = 0;
    public static final int kFrontLeftSecondEncoderPort = 1;

    public static final int kFrontRightFirstEncoderPort = 2;
    public static final int kFrontRightSecondEncoderPort = 3;

    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kMidLeftDriveMotorPort = 6;
    public static final int kMidRightDriveMotorPort = 7;
    public static final int kBackLeftDriveMotorPort = 8;
    public static final int kBackRightDriveMotorPort = 9;

    public static final double kDriveWheelRadiusMeters = .2;
    public static final double kMotorFreeSpeed = 1000;
    public static final double kMotorMaxVoltage = 12;
    public static final double kDrivetrainGearRatio = 4;
    public static final double kMaxVelocity = (kMotorFreeSpeed * Math.PI * 2 * kDriveWheelRadiusMeters) / kDrivetrainGearRatio;

    public static final double kVelocityGainSimple = kMotorMaxVoltage / kMotorFreeSpeed;
    public static final double kVelocityGain = kMotorMaxVoltage / kMaxVelocity;
  }
}
