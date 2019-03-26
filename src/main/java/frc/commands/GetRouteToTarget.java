/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.subsystems.LimelightRouteMgr;
import frc.util.TargetVecMapper;
import frc.util.Vec2d;

public class GetRouteToTarget extends Command {

  public static final double NORM_DIST = 12.0d;
  public static final double TIMEOUT = 5.0d;

  private final LimelightRouteMgr m_rteMgr;
  
  /**
   * Constructor given a Limelight route manager
   * subsystem.
   * @param rteMgr The limelight camera route manager
   * used for finding and managing the route to the
   * target
   * @param Nav The navigation subsystem
   */
  public GetRouteToTarget(LimelightRouteMgr rteMgr) {
    super(rteMgr);
    m_rteMgr = rteMgr;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("GetRouteToTarget init");
    m_rteMgr.tryToFindTarget();
    setTimeout(TIMEOUT);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!m_rteMgr.isTarget()) {
      return;
    }

    m_rteMgr.setupRouteToStdTarget(TargetVecMapper.TARGET_HEIGHT, NORM_DIST);

    Vec2d targetVec = m_rteMgr.getTargetVector();
    System.out.println("GetRouteToTarget Target:" + targetVec.toPolarString());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (isTimedOut()) {
      System.out.println("GetRouteToTarget: timed out with no target seen");
    }
    return m_rteMgr.isTarget() || isTimedOut(); 
   }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("GetRouteToTarget turning leds off");
    m_rteMgr.endFindTarget();
  }
}
