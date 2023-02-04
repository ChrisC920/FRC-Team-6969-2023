package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int FRONT_LEFT_SPARK_ID = 1;
  private static final int REAR_LEFT_SPARK_ID = 2;
  private static final int FRONT_RIGHT_SPARK_ID = 3;
  private static final int REAR_RIGHT_SPARK_ID = 4;
  private CANSparkMax m_frontLeft;
  private CANSparkMax m_rearLeft;
  private CANSparkMax m_frontRight;
  private CANSparkMax m_rearRight;
  private MotorControllerGroup m_left;
  private MotorControllerGroup m_right;

  @Override
  public void robotInit() {
    m_frontLeft = new CANSparkMax(FRONT_LEFT_SPARK_ID, MotorType.kBrushed);
    m_rearLeft = new CANSparkMax(REAR_LEFT_SPARK_ID, MotorType.kBrushed);
    m_frontRight = new CANSparkMax(FRONT_RIGHT_SPARK_ID, MotorType.kBrushed);
    m_rearRight = new CANSparkMax(REAR_RIGHT_SPARK_ID, MotorType.kBrushed);

    m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
    m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

    m_myRobot = new DifferentialDrive(m_left, m_right);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}
