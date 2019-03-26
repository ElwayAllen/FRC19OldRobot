package frc.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.util.Vec2d;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 * (by team 263)
 * @author Dan Waxman
 */
public class Limelight {
	private static NetworkTableInstance table = null;
	public static final int H_FOV = 54;
	public static final int V_FOV = 41;
	public static final double HEIGHT = 10.5;
	public static final double ANGLE_FROM_HORIZONTAL = 20d; // view angle not mounting angle, in deg
	public static final double OFFSET_FROM_CENTER = -7.0d; // offset of camera from robot center; positive to right
	public static final int VISION_PIPE = 0;							// pipeline set up for vision mode
	public static final int DRIVE_PIPE = 1;								// pipeline set up for driver mode


	/**
	 * Light modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum LightMode {
		eDefault, eOff, eBlink, eOn
	}

	/**
	 * Camera modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum CameraMode {
		eVision, eDriver
	}

	/**
	 * Initialize the Limelight and set it to the "default"
	 * operating mode.  For now, default operating mode is:
	 *  -- driver mode
	 *  -- using the "driver" pipeline
	 *  -- LEDs off
	 * 
	 */
	public Limelight() {
	}

	/**
	 * Gets whether a target is detected by the Limelight.
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean isTarget() {
		return getValue("tv").getDouble(0) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(0.00);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.00);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.00);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.00);
	}

	/**
	 * Gets target latency (ms).
	 * 
	 * @return Target latency.
	 */
	public double getTl() {
		return getValue("tl").getDouble(0.00);
	}
	
	/**
	 * Compute the distance to the currently-visible target from the camera, 
	 * based on the camera's height,its aiming angle, and the central Y angle:
	 *   distance = (targ ht - camera ht) / tan(camera angle + central y angle)
	 * This formula is inaccurate if the target height and camera height are "too close" or the
	 * sum of the camera angle and central y angle are too close to PI/2 radians (90 deg) -- so
	 * robot designers should avoid those situations!
	 * @param targetHeight Height of target above the carpet
	 * @return Distance along the carpet from the (projection of) the camera lens
	 * to the target
	 * @throws IllegalStateException if no target is in view or the "target" is at or below the
	 * horizontal
	 */
	public double getDist(double targetHeight) {
		if (!isTarget()) {
			throw new IllegalStateException("No target in view");
		}
		double centYAngle = getTy();
		System.out.println("getDist:centralYAngle:" + centYAngle);
		if (centYAngle <= -ANGLE_FROM_HORIZONTAL) {
			// Impossible case; don't divide by zero or return
			// garbage.  Throw a slightly more useful exception.
			throw new IllegalStateException("Target is at or below the horizon; can't compute distance");
		}
		return (targetHeight - HEIGHT) / Math.tan(Math.toRadians(centYAngle + ANGLE_FROM_HORIZONTAL));
	}

	/**
	 * Determine the "target vector" between the camera lens and the current target
	 * identified on the camera's screen.  The target vector is a
	 * 2-dimensional vector, in the plane of the field, whose length is the floor distance
	 * between the camera lens and the (floor projection of the) target, and whose 
	 * direction is direction is the direction from the lens to the target.
	 * The angle of the target vector is the robot's current aiming angle minus the central
	 * x angle of the target (minus, because field angles are measured CCW and the aiming
	 * angle is measured CW).  We determine the distance using getDist() [above].
	 * @param robotVec Unit vector (field-relative) in the robot's current direction
	 * @param targHeight Height of target (in units) above the floor
	 * @return Field-relative vector from camera lens to target
	 */
	public Vec2d getTargetVector(Vec2d robotVec, double targHeight) {

		double tx = getTx();
		System.out.println("getTargVec:centralXAngle:" + tx);;

		// Next, we can compute the distance to the target from the camera
		double targetDistance = getDist(targHeight);
		System.out.println("getTargVec:targetDistance:" + targetDistance);

		/* (robot angle - X angle) and distance give us the target vector */
		Vec2d targetVec = Vec2d.makePolar(targetDistance, robotVec.getTheta() - Math.toRadians(tx));
		return targetVec;
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}

		return table.getTable("limelight").getEntry(key);
	}

	/**
	 * Get a vector from the camera to the robot's center; used e.g. to
	 * help compute drive vectors.  Vector will be perpendicular to the
	 * supplied robot vector and point from the camera towards the robot
	 * vector.
	 */
	public Vec2d getCameraVector(Vec2d robotVec) {

		// Get a unit normal vector to robot vec (clockwise)
		Vec2d normVec = robotVec.getNormal();

		// Result is normal vec * -offset (minus because a camera to the left
		// of center will have a vector pointing clockwise, and vice versa)
		return normVec.mulScalar(-OFFSET_FROM_CENTER);
	}

	/**
	 * Set the Limelight to "driver" mode
	 */
	public void driverMode() {
		setLedMode(LightMode.eOff);
		setCameraMode(CameraMode.eDriver);
		setPipeline(DRIVE_PIPE);
	}

	/**
	 * Set the Limelight to "vision" mode
	 */
	public void visionMode() {
		setLedMode(LightMode.eOn);
		setCameraMode(CameraMode.eVision);
		setPipeline(VISION_PIPE);
	}
}
