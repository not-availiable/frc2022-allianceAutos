// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.AutoConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kWPI_TalonSRXAuto = "WPI_TalonSRXAuto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // auto initializations
  WPI_TalonSRX kFrontLeftDriveMotor = new WPI_TalonSRX(kFrontLeftDriveMotorPort);
  WPI_TalonSRX kMidLeftDriveMotor = new WPI_TalonSRX(kMidLeftDriveMotorPort);
  WPI_TalonSRX kBackLeftDriveMotor = new WPI_TalonSRX(kBackLeftDriveMotorPort);
  MotorControllerGroup kLeftDriveMotorControllerGroup =
      new MotorControllerGroup(kFrontLeftDriveMotor, kMidLeftDriveMotor, kBackLeftDriveMotor);

  WPI_TalonSRX kFrontRightDriveMotor = new WPI_TalonSRX(kFrontRightDriveMotorPort);
  WPI_TalonSRX kMidRightDriveMotor = new WPI_TalonSRX(kMidRightDriveMotorPort);
  WPI_TalonSRX kBackRightDriveMotor = new WPI_TalonSRX(kBackRightDriveMotorPort);
  MotorControllerGroup kRightDriveControllerGroup =
      new MotorControllerGroup(kFrontRightDriveMotor, kMidLeftDriveMotor, kBackLeftDriveMotor);
  
  Encoder kFrontlLeftDriveEncoder = new Encoder(kFrontLeftFirstEncoderPort, kFrontLeftSecondEncoderPort);
  Encoder kFrontRightDriveEncoder = new Encoder(kFrontRightFirstEncoderPort, kFrontRightSecondEncoderPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("WPI_TalonSRXAuto", kWPI_TalonSRXAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    invertRightMotors();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kWPI_TalonSRXAuto:
        WPI_TalonSRXAuto(2, 1);
        break;
      case kDefaultAuto:
      default:
        // default auto
        break;
    }
  }

  private void invertRightMotors()
  {
    kFrontRightDriveMotor.setInverted(true);
    kMidRightDriveMotor.setInverted(true);
    kBackRightDriveMotor.setInverted(true);
  }

  private void WPI_TalonSRXAuto(double meters, double speed) {
    // making sure both encoders make it past the desired distance
    double frontLeftMotorDistanceTravelledMeters =
        kFrontlLeftDriveEncoder.getDistance() * kDriveWheelRadiusMeters * Math.PI;
    double frontRightMotorDistanceTravelledMeters =
        kFrontRightDriveEncoder.getDistance() * kDriveWheelRadiusMeters * Math.PI;
    double currentDistanceTravelledMeters =
        Math.min(Math.abs(frontLeftMotorDistanceTravelledMeters), Math.abs(frontRightMotorDistanceTravelledMeters));

    // converting velocity to voltage
    double voltage = kVelocityGain * speed;

    if (currentDistanceTravelledMeters > meters) return;

    kLeftDriveMotorControllerGroup.set(
        DifferentialDrive.arcadeDriveIK(-voltage / kMotorMaxVoltage, 0, false).left);
    kRightDriveControllerGroup.set(
        DifferentialDrive.arcadeDriveIK(-voltage / kMotorMaxVoltage, 0, false).right);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
