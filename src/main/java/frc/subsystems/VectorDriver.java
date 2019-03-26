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
import frc.util.Vec2d;

/**
 * Subsystem for driving the robot according to a specified vector.
 * Given a (field-relative) vector and a velocity, this subsystem will
 * use the DriveTrain and Nav subsystems to drive the robot at the field-
 * relative angle for the distance specified by the vector, by first
 * turning the robot to the desired angle and then driving straight for
 * the specified distance.
 */
public class VectorDriver extends Subsystem {

  public static enum DrivePhase {
    eDone, eTurn, eInitDrive, eAccel, eRun, eDecel
  };

  /**
   * Our drive train
   */
  private DriveTrain m_drive;

  /**
   * Our nav subsystem
   */
  private Nav m_nav;

  /**
   * The vector we're currently driving to, or null if none
   */
  private Vec2d m_curVec;

  /**
   * The maximum velocity we're driving at
   */
  private double m_maxVel;

  /**
   * The driving state we're currently in
   */
  private DrivePhase m_phase;

  /**
   * A collision detector
   */
  private CollisionDetector m_colDet;

  private int accelSteps;
  private int runSteps;
  private int nAccelSteps;
  private int nRunSteps;

   /**
   * Constructor given drive train and nav
   * @param drive The drive train
   * @param nav The nav subsystem
   */
  public VectorDriver(DriveTrain drive, Nav nav) {
    m_drive = drive;
    m_nav = nav;
    m_colDet = new CollisionDetector(m_nav);
    m_curVec = null;
    m_maxVel = 0.0d;
    m_phase = DrivePhase.eDone;
  }

  /**
   * Start driving the specified vector at the spec'ed velocity
   * Turn to the (field-relative) angle specified by the vector,
   * then drive (as straight as possible) to the distance specified
   * by the vector.  Use a trapezoidal motion profile with the
   * specified maximum velocity and a fixed constant acceleration.
   * @param vec The vector to drive
   * @param maxVel The maximum velocity to drive at
   */
  public void startDrive(Vec2d vec, double maxVel) {
    m_curVec = vec;
    m_maxVel = maxVel;
    m_phase = DrivePhase.eTurn;

    m_drive.startTurnToAngle(Math.toDegrees(m_curVec.getTheta()));
  }

  @Override
  public void periodic() {

    /* Called every tick (20 ms) by the Scheduler.
     * If we're not doing anything just return, but if we're trying
     * to drive a vector (turn or drive straight) do the next step.
     * NOTE: Not sure it's best to do this here, but it's simpler than
     * requiring the Command to call us in its execute() method.
     */
    if (m_phase != DrivePhase.eDone) {
      m_colDet.checkForCollision();
    }

    switch (m_phase) {
      case eDone:
        return;
      
      case eTurn:
        // Turning; tell the drive to keep going
        m_drive.turnToPIDAngle();

        // And see if we're done; if so, switch state
        if (m_drive.isTurnToAngleFinished()) {
          m_phase = DrivePhase.eInitDrive;
        }
        break;

      case eInitDrive:
        initDrive();
        if (m_phase != DrivePhase.eDone) {
          m_phase = DrivePhase.eAccel;
        }
        break;

      case eAccel:
        double motorPower = (nAccelSteps == 0 ? RobotModel.startPower : m_drive.getCurrentPower());
        nAccelSteps++;
        motorPower += RobotModel.powerPerStep;
        if (motorPower > 1.0d) {
          motorPower = 1.0d;
        }
        m_drive.driveStraight(motorPower);
        if (nAccelSteps >= accelSteps) {
          m_phase = DrivePhase.eRun;
        }
        break;

      case eRun:
        motorPower = m_drive.getCurrentPower();
        nRunSteps++;
        m_drive.driveStraight(motorPower);
        if (nRunSteps >= runSteps) {
          m_phase = DrivePhase.eDecel;
        }
        break;

      case eDecel:
        motorPower = m_drive.getCurrentPower();
        if (motorPower > (RobotModel.startPower + RobotModel.powerPerStep)) {
          motorPower -= RobotModel.powerPerStep;
        }
        m_drive.driveStraight(motorPower);
        double dist = m_drive.getCurrentDistance();
        if (dist >= m_curVec.getR()) {
          finishDrive();
        }
        break;
      
    }
  }

  /**
   * Return true iff the vector drive is done
   * @return True iff the vector drive is done
   */
  public boolean isDone() {
    return (m_phase == DrivePhase.eDone);
  }

  /**
   * Reset the vector driver in case we're interrupted
   */
  public void reset()
  {
    finishDrive();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Calculate the number of acceleration and run steps required for
   * the trapezoidal motion profile, and set up the drive to start
   * driving straight for the desired distance.
   */
  private void initDrive() {
    boolean finished = ((m_maxVel <= 0) || (m_maxVel > RobotModel.maxVelocity));

    // assume trapezoid
    accelSteps = (int) Math.ceil(m_maxVel / RobotModel.velocityPerStep);

    double accelDistance = RobotModel.calculateAccelDistance(accelSteps, RobotModel.velocityPerStep, RobotModel.secPerStep);

    boolean triangularAccel = false;
    if ((2.0d * accelDistance) >= m_curVec.getR()) {

      // This is the "triangular" (vs trapezoidal) case, where we don't
      // have room to accelerate all the way to the full velocity.  In
      // this case we're going to accelerate at a constant rate for half
      // the distance, and decelerate at the same rate for the rest.
      triangularAccel = true;
      
      accelDistance = Math.floor(m_curVec.getR() / 2.0d);
      accelSteps = (int) RobotModel.calculateAccelSteps(accelDistance, RobotModel.velocityPerStep, RobotModel.secPerStep);
      accelDistance = RobotModel.calculateAccelDistance(accelSteps, RobotModel.velocityPerStep, RobotModel.secPerStep);
    }

    double runVelocity = accelSteps * RobotModel.velocityPerStep;
    double runDistance = m_curVec.getR() - (2 * accelDistance);
    double runTime = runDistance / runVelocity;
    runSteps = (int) Math.floor(runTime / RobotModel.secPerStep);
    System.out.println("Triangular acceleration: " + Boolean.toString(triangularAccel));
    System.out.println("rdist " + runDistance + " rtime " + runTime + " rsteps " + runSteps);
    System.out.println("accSteps " + accelSteps + " accDist " + accelDistance);
    nAccelSteps = 0;
    nRunSteps = 0;
    if (finished) {
      System.out.println("Drive straight for distance failed - velocity not legitimate");
      m_phase = DrivePhase.eDone;
    } else {
      m_colDet.reinitialize();
      m_drive.zeroEncoders();
      m_drive.startDriveStraight();
    }
  }

  /**
   * Terminate driving straight and reset the subsystem.
   */
  private void finishDrive() {
    m_drive.endDriveStraight();
    m_curVec = null;
    m_phase = DrivePhase.eDone;
    accelSteps = 0;
    runSteps = 0;
  }

}
