package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  // Joysticks
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  // Drive Train Variables
  private static final int leftDeviceID1 = 1;
  private static final int leftDeviceID2 = 2;
  private static final int rightDeviceID1 = 3;
  private static final int rightDeviceID2 = 4;
  private WPI_TalonSRX m_leftMotor1;
  private WPI_TalonSRX m_leftMotor2;
  private WPI_TalonSRX m_rightMotor1;
  private WPI_TalonSRX m_rightMotor2;

  // Arm
  private CANSparkMax m_arm;
  private RelativeEncoder arm_encoder;
  private static final int armDeviceID = 5;

  // Grabber
  private CANSparkMax m_grabber;
  private RelativeEncoder grabber_encoder;
  private static final int grabberDeviceID = 6;

  // Gyro
  private AHRS ahrs;
  private boolean autoBalanceXMode;
  private boolean autoBalanceYMode;
  static final double kOffBalanceAngleThresholdDegrees = 2;
  static final double kOonBalanceAngleThresholdDegrees = 1;
  private boolean hasStartedGyro = false;

  // drive trains
  private DifferentialDrive m_myRobot;
  private MecanumDrive m_myRobot2;

  // timer used for autonomous
  private final Timer m_timer = new Timer();

  // LIMIT CONSTANTS
  private final int grabberCubeLim = -12;
  private final int grabberConeLim = 2;
  private final int armAutoLim = 145;

  // AUTONOMOUS TIME USAGE VARIABLES
  private int autoGrabberVar = 0;
  private int autoArmVar = 0;

  // sets "motor" To brake mode, will hold position when stopped but cannot be
  // driven
  public void setBrake(CANSparkMax motor) {
    try {
      motor.setIdleMode(IdleMode.kBrake);
    } catch (Exception e) {
      DriverStation.reportError("Error setting to brake mode", true);
    }
  }

  // sets "motor" To brake mode, will hold position when stopped but cannot be
  // driven
  public void setBrake(WPI_TalonSRX motor) {
    try {
      motor.setNeutralMode(NeutralMode.Brake);
    } catch (Exception e) {
      DriverStation.reportError("Error setting to brake mode", true);
    }
  }

  // sets "motor" to coast mode, won't hold position when stopped but can be
  // driven
  public void setCoast(CANSparkMax motor) {
    try {
      motor.setIdleMode(IdleMode.kCoast);
    } catch (Exception e) {
      DriverStation.reportError("Error setting to coast mode", true);
    }
  }

  // sets "motor" to coast mode, won't hold position when stopped but can be
  // driven
  public void setCoast(WPI_TalonSRX motor) {
    try {
      motor.setNeutralMode(NeutralMode.Coast);
    } catch (Exception e) {
      DriverStation.reportError("Error setting to coast mode", true);
    }
  }

  // runs once when teleop starts
  @Override
  public void robotInit() {
    // Left side drive train motor group
    m_leftMotor1 = new WPI_TalonSRX(leftDeviceID1);
    m_leftMotor2 = new WPI_TalonSRX(leftDeviceID2);
    MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    // Right side drive train motor group
    m_rightMotor1 = new WPI_TalonSRX(rightDeviceID1);
    m_rightMotor2 = new WPI_TalonSRX(rightDeviceID2);
    MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

    // Arm
    m_arm = new CANSparkMax(armDeviceID, MotorType.kBrushless);
    arm_encoder = m_arm.getEncoder();

    // Grabber
    m_grabber = new CANSparkMax(grabberDeviceID, MotorType.kBrushless);
    grabber_encoder = m_grabber.getEncoder();

    // Reset factory defauts just in case
    m_leftMotor1.configFactoryDefault();
    m_leftMotor2.configFactoryDefault();
    m_rightMotor1.configFactoryDefault();
    m_rightMotor2.configFactoryDefault();
    m_arm.restoreFactoryDefaults();
    m_grabber.restoreFactoryDefaults();

    // Inversions as neccessary
    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);

    // Setting drive train
    m_myRobot = new DifferentialDrive(m_left, m_right);
    m_myRobot2 = new MecanumDrive(m_leftMotor1, m_leftMotor2, m_rightMotor1, m_rightMotor2);
    m_myRobot.setExpiration(0.1);
    m_myRobot2.setExpiration(0.1);

    // Joysticks
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    arm_encoder.setPosition(0);
    grabber_encoder.setPosition(0);
  
    // Gyro
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

  }

  // runs initially when autonomous starts
  @Override
  public void autonomousInit() {
    m_timer.reset();
    hasStartedGyro = false;

    // set motors to brake mode
    setCoast(m_leftMotor1);
    setCoast(m_leftMotor2);
    setCoast(m_rightMotor1);
    setCoast(m_rightMotor2);
    setBrake(m_grabber);
    autoGrabberVar = 0;
    autoArmVar = 0;
    hasStarted = false;
    inRange = false;
    autoBalanceXMode = false;
    autoBalanceYMode = false;
  }

  // runs periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    autoTaxi();
    // autoMiddleNoBalance();
    // autoMiddleBalance();
    // autonomousGyro();
    // autonomousConeScore();
  }

  // autonomous short taxi side
  public void autoTaxi() {
    if (autoArmVar == 0 || autoArmVar == 1)
      autonomousConeScore();
    else if (autoArmVar == 2)
      autonomousTaxiB();
  }

  // autonomous middle no charging station
  public void autoMiddleNoBalance() {
    autonomousConeScore();
  }

  // autonomous middle with charging station
  public void autoMiddleBalance() {
    if (autoArmVar == 0 || autoArmVar == 1)
      autonomousConeScore();
    else if (autoArmVar == 2) {
      autonomousGyro();
    }
  }

  // helper function to score cone
  public void autonomousConeScore() {
    // cone score
    // arm open
    if (arm_encoder.getPosition() < armAutoLim && autoArmVar == 0) {
      m_arm.set(0.75);
      if (arm_encoder.getPosition() < 5) {
        setCoast(m_grabber);
        m_grabber.set(0.2);
      } else {
        setBrake(m_grabber);
      }
    }

    // grabber open and close
    else if (autoArmVar == 0) {
      if (autoGrabberVar == 0) {
        m_arm.set(0);
        setBrake(m_arm);
      }

      if (grabber_encoder.getPosition() > -35 && autoGrabberVar == 0) {
        setCoast(m_grabber);
        m_grabber.set(-0.45);
      } else if (autoGrabberVar == 0) {
        m_grabber.set(0);
        autoGrabberVar = 1;
      } else if (grabber_encoder.getPosition() < grabberConeLim && autoGrabberVar == 1) {
        m_grabber.set(0.45);
        // setCoast(m_arm);
        // m_arm.set(-0.8);
      } else {
        m_grabber.set(0);
        setBrake(m_grabber);
        autoGrabberVar = 2;
        autoArmVar = 1;
      }
    }
    // arm close
    else if (arm_encoder.getPosition() > 16 && autoArmVar == 1) {
      m_arm.set(-0.75);
    }
    // finish with all in brake
    else {
      m_arm.set(0);
      setBrake(m_arm);
      autoArmVar = 2;
    }
  }

  // helper function to taxi backwards
  public void autonomousTaxiB() {
    m_timer.start();
    if (m_timer.get() < 2.7) {
      m_myRobot2.driveCartesian(-0.4, 0, 0);
    }
  }

  // helper function to taxi forwards
  public void autonomousTaxiF() {
    m_timer.start();
    if (m_timer.get() < 2.7) {
      m_myRobot2.driveCartesian(0.4, 0, 0);
    }
  }

  // Helper function to get bot to balance on charge station
  public void autonomousGyro() {
    setCoast(m_leftMotor1);
    setCoast(m_leftMotor2);
    setCoast(m_rightMotor1);
    setCoast(m_rightMotor2);

    double xAxisRate = 0;
    double yAxisRate = 0;
    double rollAngleDegrees = ahrs.getPitch();
    double pitchAngleDegrees = ahrs.getRoll();
    if (!hasStartedGyro) {
      m_myRobot2.driveCartesian(-0.5, 0, 0);
      if (pitchAngleDegrees <= -12) {
        m_timer.start();
      }
    } 
    if (m_timer.get() > 1) {
      hasStartedGyro = true;
    }
    else {
      if (!autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
        autoBalanceXMode = true;
      } else if (autoBalanceXMode && (Math.abs(pitchAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
        autoBalanceXMode = false;
      }
      if (!autoBalanceYMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
        autoBalanceYMode = true;
      } else if (autoBalanceYMode && (Math.abs(pitchAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
        autoBalanceYMode = false;
      }

      if (autoBalanceXMode) {
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        xAxisRate = Math.sin(pitchAngleRadians) * -1;
      }
      if (autoBalanceYMode) {
        double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
        yAxisRate = Math.sin(rollAngleRadians) * -1;
      }
      if (xAxisRate > -0.05 && xAxisRate < 0.05) {
        xAxisRate = 0;
      }

      try {
        m_myRobot2.driveCartesian(-xAxisRate * 1.5, 0, 0);
      } catch (RuntimeException ex) {
        String err_string = "Drive system error: " + ex.getMessage();
        DriverStation.reportError(err_string, true);
      }

    }
    SmartDashboard.putNumber("X", ahrs.getRoll());
    SmartDashboard.putNumber("Y", xAxisRate);
    // SmartDashboard.putNumber("angle", );
  }

  // Helper function to rotate bot Theta degrees
  boolean hasStarted = false;
  boolean inRange = false;
  Timer backUpTime = new Timer();
  Timer tempTimer = new Timer();

  public void autonomousRotate(double angle, double degreesOfFreedom) {
    if (!hasStarted) {
      ahrs.reset();
      hasStarted = true;
      backUpTime.reset();
      backUpTime.start();
    } else {
      if (backUpTime.get() < 1) {
        m_myRobot2.driveCartesian(-0.2, 0, 0);
      } else {
        if (ahrs.getAngle() >= -angle) {
          m_myRobot2.driveCartesian(0, 0, 0.5);
        } else if (ahrs.getAngle() < -angle && !inRange) {
          m_myRobot2.driveCartesian(0, 0, -0.3);
          tempTimer.start();
        } else {
          m_myRobot2.driveCartesian(0, 0, 0);

        }
      }
    }
    if (tempTimer.get() > 1.5) {
      autoArmVar = 3;
    }

    if (-ahrs.getAngle() < (angle + degreesOfFreedom) && -ahrs.getAngle() > (angle - degreesOfFreedom)) {
      inRange = true;
    } else {
      inRange = false;
    }
    SmartDashboard.putNumber("angle", ahrs.getAngle());
  }

  // runs once at end of autonomous
  @Override
  public void autonomousExit() {
    m_myRobot2.driveCartesian(0, 0, 0);
    setBrake(m_leftMotor1);
    setBrake(m_leftMotor2);
    setBrake(m_rightMotor1);
    setBrake(m_leftMotor2);
  }

  // runs once when teleop starts
  @Override
  public void teleopInit() {
    arm_encoder.setPosition(0);
    grabber_encoder.setPosition(0);
    setCoast(m_leftMotor1);
    setCoast(m_leftMotor2);
    setCoast(m_rightMotor1);
    setCoast(m_rightMotor2);
  }

  // during teleop this runs every ms
  @Override
  public void teleopPeriodic() {
    // grabber open
    if (arm_encoder.getPosition() > 35 && grabber_encoder.getPosition() > -100 && m_rightStick.getTrigger()) {
      setCoast(m_grabber);
      m_grabber.set(-0.8);
    }
    // grabber close
    else if (grabber_encoder.getPosition() < grabberConeLim
        && (m_rightStick.getRawButton(5) || m_rightStick.getRawButton(3))) {
      setCoast(m_grabber);
      m_grabber.set(0.8);
    } else if (grabber_encoder.getPosition() < grabberCubeLim && m_rightStick.getRawButton(2)) {
      setCoast(m_grabber);
      m_grabber.set(0.8);
    } else {
      m_grabber.set(0);
      setBrake(m_grabber);
    }

    // Arm movement
    if ((m_rightStick.getY() > 0.05 || (m_rightStick.getY() < -0.05))) {
      setCoast(m_arm);

      // arm limits
      if (arm_encoder.getPosition() > 13 && m_rightStick.getY() > 0.05)
        m_arm.set(-m_rightStick.getY() * 1.1);
      else if (arm_encoder.getPosition() < 209 && m_rightStick.getY() < -0.05)
        m_arm.set(-m_rightStick.getY() * 1.1);
      else {
        m_arm.set(0);
        setBrake(m_arm);
      }
    } else {
      m_arm.set(0);
      setBrake(m_arm);
    }

    // drive train (arcade drive)
    if (!m_leftStick.getTrigger()) {
      m_myRobot.arcadeDrive(-m_leftStick.getY(), m_leftStick.getX() * 1.3);
    } else if (m_leftStick.getTrigger()) {
      m_myRobot.arcadeDrive(-m_leftStick.getY() * 1.5, m_leftStick.getX() * 1.5);
    }

    // output values for debugging
    SmartDashboard.putNumber("right joystick", m_rightStick.getY());
    SmartDashboard.putNumber("right joystick X", m_rightStick.getX());
    SmartDashboard.putNumber("Arm Position", arm_encoder.getPosition());
    SmartDashboard.putNumber("Grabber Position", grabber_encoder.getPosition());
  }

  @Override
  public void teleopExit() {
    m_myRobot.arcadeDrive(0, 0);
    setBrake(m_leftMotor1);
    setBrake(m_leftMotor2);
    setBrake(m_rightMotor1);
    setBrake(m_rightMotor2);
  }
}