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

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.subsystems.DriveTrain;
import frc.subsystems.Limelight;
import frc.subsystems.LimelightRouteMgr;
import frc.subsystems.Nav;
import frc.subsystems.TestRouteMgr;
import frc.subsystems.VectorDriver;
import frc.commands.DriveLeftCommand;
import frc.commands.DriveRightCommand;
import frc.commands.DriveStraightCommand;
import frc.commands.DriveWithJoystick;
import frc.commands.GetRouteToTarget;
import frc.commands.GetRouteToRocketTarget;
import frc.commands.DriveRouteToTarget;
import frc.commands.ResetGyroCommand;
import frc.commands.AbortCommand;
import frc.commands.DriveRoute;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private double m_drivePower = 0.2d;   // default drive power
  private double m_maxTime = 10.0;      // max command time (secs)
  private double m_driveAngle = 90.0;   // angle to drive to (deg)
  private double m_driveDist = 24.0;    // distance to drive to (in)
  private double m_driveVel = 48.0;     // max drive velociy (in/sec)
  private Robot m_robot;
  private XboxController m_ctl;
  private AbortCommand m_abortCmd;
  private JoystickButton aBut;
  private JoystickButton bBut;
  private JoystickButton xBut;
  private JoystickButton yBut;
  
  public OI(Robot robot) {
    m_robot = robot;
    m_ctl = new XboxController(0);
    m_abortCmd = new AbortCommand();    // save for use later

    Nav nav = m_robot.getNav();
    DriveTrain dtr = m_robot.getDriveTrain();
    Limelight cam = m_robot.getCam();
    LimelightRouteMgr rteMgr = m_robot.getRouteMgr();
    TestRouteMgr tRteMgr = new TestRouteMgr();
    tRteMgr.initRoute();
    VectorDriver vecDrv = m_robot.getVectorDriver();

    m_drivePower = SmartDashboard.getNumber("Drive Power", m_drivePower);
    m_maxTime = SmartDashboard.getNumber("Max Time", m_maxTime);
    m_driveAngle = SmartDashboard.getNumber("Drive Angle", m_driveAngle);
    m_driveDist = SmartDashboard.getNumber("Drive Dist", m_driveDist);
    m_driveVel = SmartDashboard.getNumber("Drive Vel", m_driveVel);

    aBut = new JoystickButton(m_ctl, 1);
    bBut = new JoystickButton(m_ctl, 2);
    xBut = new JoystickButton(m_ctl, 3);
    yBut = new JoystickButton(m_ctl, 4);
    SmartDashboard.putData("Abort All", m_abortCmd);
    aBut.whenPressed(new DriveRoute(tRteMgr, vecDrv, m_driveVel));
    bBut.whenPressed(new ResetGyroCommand(nav));
    yBut.whenPressed(new GetRouteToTarget(rteMgr));
    aBut.whenPressed(new DriveRouteToTarget(rteMgr, vecDrv, m_driveVel));
    xBut.whenPressed(m_abortCmd);
    SmartDashboard.putData("Drive Left", new DriveLeftCommand(m_maxTime, dtr));
    SmartDashboard.putData("Drive Right", new DriveRightCommand(m_maxTime, dtr));
    SmartDashboard.putData("DriveStraight", new DriveStraightCommand(m_maxTime, dtr));
    SmartDashboard.putData("Route", new GetRouteToTarget(rteMgr));
    SmartDashboard.putData("RocketRte", new GetRouteToRocketTarget(nav, cam));
    SmartDashboard.putData("Drive to Target", new DriveRouteToTarget(rteMgr, vecDrv, m_driveVel));
    SmartDashboard.putData("Vector Drive", new DriveRoute(tRteMgr, vecDrv, m_driveVel));
    SmartDashboard.putData("Reset gyro", new ResetGyroCommand(nav));

    dtr.setDefaultCommand(new DriveWithJoystick(dtr, m_ctl));
  }
}
