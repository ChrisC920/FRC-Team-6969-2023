package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  // drive train
  private static final int leftDeviceID1 = 1;
  private static final int leftDeviceID2 = 2;
  private static final int rightDeviceID1 = 3;
  private static final int rightDeviceID2 = 4;
  private static final int armDeviceID = 5;
  private static final int grabberDeviceID = 6;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;

  // arm/grabber
  private CANSparkMax m_arm;
  private CANSparkMax m_grabber;
  private RelativeEncoder arm_encoder;
  private RelativeEncoder grabber_encoder;

  // gyro
  AHRS ahrs;
  double kP = 1;
  boolean autoBalanceXMode;
  boolean autoBalanceYMode;
  private DifferentialDrive m_myRobot;

  // func variables
  private int armCount = 0;

  @Override
  public void robotInit() {
    // left side drive train motor group
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
    MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    // right side drive train motor group
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);
    MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

    // reset factory defauts just in case
    m_leftMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_arm.restoreFactoryDefaults();
    m_grabber.restoreFactoryDefaults();

    // encoders
    arm_encoder = m_arm.getEncoder();
    arm_encoder.setPositionConversionFactor(360);
    grabber_encoder = m_grabber.getEncoder();
    grabber_encoder.setPositionConversionFactor(360);

    // inversions as neccessary
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);

    // setting drive train
    m_myRobot = new DifferentialDrive(m_left, m_right);
    m_myRobot.setExpiration(0.1);

    // joysticks
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    // gyro bullshit
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

  }

  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees = 5;

  @Override
  public void autonomousInit() {
    // m_myRobot = new MecanumDrive(m_leftMotor1, m_leftMotor2, m_rightMotor1,
    // m_rightMotor2);

  }

  @Override
  public void autonomousPeriodic() {
    // double xAxisRate = m_leftStick.getX();
    // double yAxisRate = m_leftStick.getY();
    // double pitchAngleDegrees = ahrs.getPitch();
    // double rollAngleDegrees = ahrs.getRoll();

    // if (!autoBalanceXMode && (Math.abs(pitchAngleDegrees) >=
    // Math.abs(kOffBalanceAngleThresholdDegrees))) {
    // autoBalanceXMode = true;
    // } else if (autoBalanceXMode && (Math.abs(pitchAngleDegrees) <=
    // Math.abs(kOonBalanceAngleThresholdDegrees))) {
    // autoBalanceXMode = false;
    // }
    // if (!autoBalanceYMode && (Math.abs(pitchAngleDegrees) >=
    // Math.abs(kOffBalanceAngleThresholdDegrees))) {
    // autoBalanceYMode = true;
    // } else if (autoBalanceYMode && (Math.abs(pitchAngleDegrees) <=
    // Math.abs(kOonBalanceAngleThresholdDegrees))) {
    // autoBalanceYMode = false;
    // }

    // // Control drive system automatically,
    // // driving in reverse direction of pitch/roll angle,
    // // with a magnitude based upon the angle

    // if (autoBalanceXMode) {
    // double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
    // xAxisRate = Math.sin(pitchAngleRadians) * -1;
    // }
    // if (autoBalanceYMode) {
    // double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
    // yAxisRate = Math.sin(rollAngleRadians) * -1;
    // }

    // try {
    // m_myRobot.driveCartesian(xAxisRate, yAxisRate, m_leftStick.getTwist(),
    // ahrs.getRotation2d());
    // } catch (RuntimeException ex) {
    // String err_string = "Drive system error: " + ex.getMessage();
    // DriverStation.reportError(err_string, true);
    // }
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(m_leftStick.getY(), m_rightStick.getX());
    if (m_rightStick.getTriggerPressed() && arm_encoder.getPosition() <= 30) {
      arm_encoder.setPosition(arm_encoder.getPosition() + 1);
    } else if (m_leftStick.getTriggerPressed() && arm_encoder.getPosition() >= -30) {
      arm_encoder.setPosition(arm_encoder.getPosition() - 1);
    }
  }
}