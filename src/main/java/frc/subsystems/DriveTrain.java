/*
 * The MIT License
 *
 * Copyright 2019 lwa.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package frc.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotMap;
import frc.subsystems.RobotModel;


/**
 * Simple drive train for driving the kitbot during test mode
 * from the Driver's Station only.  
 * Now includes support for straight driving using a simple gyro.
 */
public class DriveTrain extends Subsystem implements PIDOutput {
  private Nav m_nav;
  // The motor controllers; rear controllers are masters
  private WPI_TalonSRX m_leftBack;
  private WPI_TalonSRX m_rightBack;
  private Spark m_leftFront;
  private Spark m_rightFront;
  private SpeedControllerGroup m_leftGroup;
  private SpeedControllerGroup m_rightGroup;
  private DifferentialDrive m_drive;

  private PIDController m_turnController;
  private double m_rotateToAngleRate;       // communication between pid control and rotator
  private double m_kTargetAngleDegrees; // angle we're turning to, if any

  static double kP = 0.025;
  static double kI = 0.00;
  static double kD = 0.1;
  static double kF = 0.00;
  static final double kToleranceDegrees = 2.0; 
  static final double kToleranceSpeed = 200; 
  static public final double kEncoderUnitsPerRevolution = 1440; 
  public final static int kTimeoutMs = 30;
   
  // For now, limit max voltage to motors to limit damage
  public double PEAK_OUTPUT = 1.0d;

  /**
   * Empty constructor
   */
  public DriveTrain(Nav nav) {
    m_nav = nav;
    m_leftBack = new WPI_TalonSRX(RobotMap.leftBackTalon);
    m_rightBack = new WPI_TalonSRX(RobotMap.rightBackTalon);
    m_leftFront = new Spark(RobotMap.leftFrontSpark);
    m_rightFront = new Spark(RobotMap.rightFrontSpark);

    // Initialize and set up the motor controllers here
    m_leftBack.configFactoryDefault();
    m_rightBack.configFactoryDefault();
    m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setInverted(true);
    m_leftFront.setInverted(true);
    m_rightFront.setInverted(true);
    m_leftBack.configPeakOutputForward(PEAK_OUTPUT);
    m_rightBack.configPeakOutputForward(PEAK_OUTPUT);
    m_leftBack.configPeakOutputReverse(-PEAK_OUTPUT);
    m_rightBack.configPeakOutputReverse(-PEAK_OUTPUT);
    m_leftBack.setSensorPhase(true);
    m_rightBack.setSensorPhase(false);
    
    m_leftGroup = new SpeedControllerGroup(m_leftBack, m_leftFront);
    m_rightGroup = new SpeedControllerGroup(m_rightBack, m_rightFront);

    m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

    // Init turn controller for PID controller turning and straight drive
    m_rotateToAngleRate = 0.0;
    m_kTargetAngleDegrees = m_nav.getYaw();
    m_turnController = new PIDController(kP, kI, kD, kF, nav.getPIDSource(), this);
    m_turnController.setInputRange(-180.0f,  180.0f);
    m_turnController.setOutputRange(-0.5, 0.5);
    m_turnController.setAbsoluteTolerance(kToleranceDegrees);
    m_turnController.setContinuous(true);
    m_turnController.disable();

    /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
    /* tuning of the Turn Controller's P, I and D coefficients.            */
    /* Typically, only the P value needs to be modified.                   */
    m_turnController.setName("DriveTrain", "TurnController");
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Drive the left side (only) at the specified motor power
   * @param power Motor power (-1.0 .. 1.0)
   */
  public void driveLeft(double motorPower) {
    if ((motorPower < -1.0) || (motorPower > 1.0)) {
      throw new IllegalArgumentException("motor power out of range " + motorPower);
    }
    m_leftGroup.set(motorPower);
    m_rightGroup.stopMotor();
  }

  /**
   * Drive the right side (only) at the specified motor power
   * @param power Motor power (-1.0 .. 1.0)
   */
  public void driveRight(double motorPower) {
    if ((motorPower < -1.0) || (motorPower > 1.0)) {
      throw new IllegalArgumentException("motor power out of range " + motorPower);
    }
    m_rightGroup.set(motorPower);
    m_leftGroup.stopMotor();
  }

  /**
   * Drive the drive as straight as (reasonably) possible without
   * using a gyro or other PID compensation device.
   * @param power Motor power (-1.0 .. 1.0)
   */
  public void DriveAhead(double motorPower) {
    m_drive.tankDrive(motorPower, motorPower);
  }

  /**
   * Tank drive the robot using the specified motor powers for
   * each side.
   * @param leftPower Power to left side
   * @param rightPower power to right side
   */
  public void tankDrive(double leftPower, double rightPower) {
    m_drive.tankDrive(leftPower, rightPower);
  }

  public double getCurrentPower() {
    // Used in driveStraight primarily; use the greater of the two
    // motors
    double leftPower = m_leftGroup.get();
    double rightPower = m_rightGroup.get();
    return(Math.abs(leftPower) > Math.abs(rightPower) ? leftPower : rightPower);
  }

  public void zeroEncoders() {
      m_leftBack.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
      m_rightBack.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    }

  public double getCurrentDistance() {
    double avgUnits = (double) m_leftBack.getSelectedSensorPosition() + (double) m_rightBack.getSelectedSensorPosition() / 2.0d;
    return (RobotModel.distancePerRevolution * avgUnits) / kEncoderUnitsPerRevolution;
	}


  /**
   * Stop all motors
   */
  public void stopAll() {
    m_rightBack.stopMotor();
    m_leftBack.stopMotor();
    m_turnController.disable();
  }

  // PID controlled methods

  public void startTurnToAngle(double degrees) {
    System.out.println("Start turn to angle " + degrees + " at yaw " + m_nav.getYaw());
    m_kTargetAngleDegrees = Nav.fieldAngleToYaw(degrees);
    m_turnController.setSetpoint(m_kTargetAngleDegrees);
    m_rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
    m_turnController.enable();
  }

  // startTurnToAngle() should already have been called to set up!
  public void turnToPIDAngle() {
		double leftStickValue = m_rotateToAngleRate / 2.0d;
		double rightStickValue = -(m_rotateToAngleRate / 2.0d);
    m_drive.tankDrive(leftStickValue, rightStickValue);
  }

  public boolean isTurnToAngleFinished() {
		double angleDifference = Math.abs(m_nav.getYaw() - m_kTargetAngleDegrees);
		double totalSpeed = Math.abs(m_leftBack.getSelectedSensorVelocity()) + Math.abs(m_rightBack.getSelectedSensorVelocity());

    boolean res = (angleDifference < kToleranceDegrees) && (totalSpeed < kToleranceSpeed);
    if (res) {
      System.out.println("End turn to angle at yaw " + m_nav.getYaw());
      m_drive.stopMotor();
      m_turnController.disable();
    }
    return res;
  }

  public void startDriveStraight() {
    double yaw = m_nav.getYaw();
    System.out.println("Start drive straight at yaw " + yaw);
    m_turnController.setSetpoint(yaw);
    m_rotateToAngleRate = 0.0d;
    m_turnController.enable();
  }

  // startDriveStraight() should already have been called!
  public void driveStraight(double power) {
    if (Math.abs(power) > 1.0d) {
      throw new IllegalArgumentException("Illegal motor power to driveStraight " + power);
    }
    double leftStickValue = power + (m_rotateToAngleRate / 2.0d);
		double rightStickValue = power - (m_rotateToAngleRate / 2.0d);

		if ((leftStickValue < -1.0d) || (leftStickValue > 1.0d)) {
			rightStickValue = power - m_rotateToAngleRate;
			leftStickValue = power;
		} else if ((rightStickValue < -1.0d) || (rightStickValue > 1.0d)) {
      leftStickValue = power + m_rotateToAngleRate;
      rightStickValue = power;
    }

    m_drive.tankDrive(leftStickValue, rightStickValue);
  }

  public void endDriveStraight() {
    m_drive.stopMotor();
    m_turnController.disable();
  }

  // Methods from PIDOutput

  @Override
  public void pidWrite(double output) {
    m_rotateToAngleRate = output;
  }

}
