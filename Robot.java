package org.usfirst.frc.team835.robot;

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	// Robot Drive Variable
	private RobotDrive myRobot;
	// Motor Controllers
	private TalonSRX rightB, leftB, rightF, leftF, winch, collector, shooter;
	// Camera Resolution
	private final int xRes = 240;
	private final int yRes = 360;
	// Gyroscope (CLOCKWISE IS POSITIVE)
	private ADXRS450_Gyro gyro;
	private final double GYRO_CONST = 0.04;
	private double ORIGINAL_ANGLE;
	// Ultrasonic Sensor
	Ultrasonic uss;
	// Pneumatics
	DoubleSolenoid ds;
	Compressor compressor;
	// Joysticks
	private Joystick whiteR, whiteL;
	// Network Table
	private NetworkTable table;
	// Dashboard Sendables
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoModeSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	// Other variables
	private double shooter_speed, belt_speed, bearing, distance;
	private double X, Y, Z;
	private boolean isReversed;
	SideGearAuto sideStep;
	CenterGearAuto centerStep;
	private double time_elapsed;
	private double autonomousStartTime, currenttime;

	public Robot() {
		NetworkTable.setTeam(835);
		table = NetworkTable.getTable("GRIP/targets");
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// Retrieve NetworkTables
		NetworkTable.setIPAddress("127.0.0.1");
		table = NetworkTable.getTable("GRIP/targets");
		// Instantiate Motor Controllers
		rightB = new TalonSRX(0);
		leftB = new TalonSRX(1);
		rightF = new TalonSRX(2);
		leftF = new TalonSRX(3);
		// winch = new TalonSRX(6);
		collector = new TalonSRX(4);
		shooter = new TalonSRX(5);
		// Reverse input for left motor Controllers
		leftB.setInverted(true);
		leftF.setInverted(true);
		// Instantiate Gyro, calibrate and reset
		// The ADXRS450 Gyro Class is specific to the gyro that is currently
		// being used on the robot.
		// Read WPILIB's Java Database for classes for other Gyros
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		gyro.reset();
		ORIGINAL_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGINAL_ANGLE);
		// Instantiate Pnuematics Components
		ds = new DoubleSolenoid(0, 1);
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);
		// Instantiate Ultrasonic Sensor. The first parameter is the analog Ping
		// Channel. The second parameter is the Analog Echo Channel
		uss = new Ultrasonic(0, 1);
		uss.setAutomaticMode(true);
		// Instantiate Joysticks
		whiteR = new Joystick(1);
		whiteL = new Joystick(0);
		// Instantiate RobotDrive with 4 Motor Controllers
		myRobot = new RobotDrive(leftF, leftB, rightF, rightB);
		// Sendable Chooser for Autonomous modes
		chooser.addDefault("0. Default: Cross Baseline", defaultAuto);
		chooser.addObject("1. Gear Position Left", "leftG");
		chooser.addObject("2. Gear Position Center", "centerG");
		chooser.addObject("3. Gear Position Right", "rightG");
		chooser.addObject("4. Shoot In Low Boiler", "lowB");
		chooser.addObject("5. Shoot In High Boiler", "highB");
		SmartDashboard.putData("Autonomous choices", chooser);
	}

	int loopTimer;

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	public void autonomousInit() {
		autoModeSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto Mode selected: " + autoModeSelected);
		SmartDashboard.putString("Autonomous Mode", autoModeSelected);
		// Pre-auto stuff
		gyro.calibrate();
		ORIGINAL_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGINAL_ANGLE);
		gyro.reset();
		// Starts with forward drive system (collector in front)
		isReversed = true;
		loopTimer = 0;
		autonomousStartTime = System.currentTimeMillis();
	}

	public void autonomousPeriodic() {
		if (time_elapsed > 15) {
			return;
		}
		boolean RobotOnLeft, vision;
		sensors();
		switch (autoModeSelected) {// Enums look cool
		case "centerG":
			centerGearAuto();
			break;
		case "leftG":
			if (time_elapsed < 3) {
				vision = false;
				sideStep = SideGearAuto.DRIVE_STRAIGHT;
			} else if (time_elapsed >= 2 && time_elapsed < 5) {
				vision = false;
				sideStep = SideGearAuto.ROTATE_RIGHT;
			} else if ((time_elapsed >= 5 && time_elapsed < 8) || (bearing < 75 && bearing > 45)) {
				vision = false;
				sideStep = SideGearAuto.FAR_APPROACH;
			} else if (time_elapsed >= 8 && time_elapsed < 10 || distance < 20) {
				vision = true;
				sideStep = SideGearAuto.CLOSE_APPROACH;
			} else if (time_elapsed >= 10 || distance < 13) {
				vision = true;
				sideStep = SideGearAuto.STOP;
			} else { // should never happen
				vision = false;
				System.out.println("BIG ERROR");
			}
			leftGearAuto(RobotOnLeft = true, vision);
			break;
		case "rightG":
			if (time_elapsed < 3) {
				vision = false;
				sideStep = SideGearAuto.DRIVE_STRAIGHT;
			} else if (time_elapsed >= 2 && time_elapsed < 5) {
				vision = false;
				sideStep = SideGearAuto.ROTATE_LEFT;
			} else if ((time_elapsed >= 5 && time_elapsed < 8) || (bearing < -45 && bearing > -75)) {
				vision = false;
				sideStep = SideGearAuto.FAR_APPROACH;
			} else if (time_elapsed >= 8 && time_elapsed < 10 || distance < 20) {
				vision = true;
				sideStep = SideGearAuto.CLOSE_APPROACH;
			} else if (time_elapsed >= 10 || distance < 13) {
				vision = true;
				sideStep = SideGearAuto.STOP;
			} else { // should never happen
				vision = false;
				System.out.println("BIG ERROR");
			}
			leftGearAuto(RobotOnLeft = false, vision);
			break;
		case "lowB": // get shooter value for shooting at a range
			break;
		case "highB": // probably not going to do this one
			break;
		default:
			baseLineAuto();
			break;
		}
		loopTimer++;
	}

	/**
	 * Autonomous Routine for putting gear in side peg. Parameter onLeft used to
	 * determine which way robot should turn while approaching
	 */
	void leftGearAuto(boolean RobotOnLeft, boolean vision) {
		double x = xRes;
		double curve = 0;
		if (vision) {
			double[] centerX = table.getNumberArray("centerX", new double[0]);

			if (centerX.length >= 2) {
				x = (centerX[0] + centerX[1]) / 2.0;
			} else {
				x = xRes / 2;
			}
			curve = curveToCenter(x);
		}

		if (x < 0.4 * xRes || x > .6 * xRes) {
			sideStep = SideGearAuto.OFF_CENTER;
		}

		if (sideStep == SideGearAuto.OFF_CENTER) {
			X = (x - xRes / 2) / xRes;
			Y = 0;
			Z = 0;
		} else if (sideStep == SideGearAuto.DRIVE_STRAIGHT) {
			X = 0;
			Y = 0.9;
			Z = 0;
			bearing *= 1.5;
		} else if (sideStep == SideGearAuto.ROTATE_LEFT) {
			X = 0;
			Y = 0;
			Z = rotateTo(-60);
		} else if (sideStep == SideGearAuto.ROTATE_RIGHT) {
			X = 0;
			Y = 0;
			Z = rotateTo(60);
		} else if (sideStep == SideGearAuto.FAR_APPROACH) {
			X = 0;
			Y = 0.6;
			Z = curve;
			bearing *= 1.2;
		} else if (sideStep == SideGearAuto.CLOSE_APPROACH) {
			X = 0;
			Y = 0.3;
			Z = curve;
		} else if (sideStep == SideGearAuto.STOP) {
			X = 0;
			Y = 0;
			Z = 0;
			bearing = 0;
		}
		myRobot.mecanumDrive_Cartesian(X, Y, Z, bearing);
	}

	/**
	 * Autonomous Routine for putting gear in center peg
	 */
	void centerGearAuto() {
		double[] centerX = table.getNumberArray("centerY", new double[0]);
		double x;
		if (centerX.length >= 2) {
			x = (centerX[0] + centerX[1]) / 2.0;
		} else {
			x = xRes / 2;
		}
		double curve = -curveToCenter(x) * .7;
		SmartDashboard.putNumber("distance", distance);
		SmartDashboard.putNumber("bearing", bearing);
		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("curve", curve);
		if (distance > 50) {
			myRobot.mecanumDrive_Cartesian(0, -0.5, curve, bearing);
		} else if (distance > 11.5) {
			if (x < .4 * xRes) {
				myRobot.mecanumDrive_Cartesian(-0.3, 0, curve, bearing);
			} else if (x > 0.6 * xRes) {
				myRobot.mecanumDrive_Cartesian(0.3, 0, curve, bearing);
			} else {
				myRobot.mecanumDrive_Cartesian(0, -0.3, curve, bearing);
			}
		} else if (distance <= 11.5) {
			rotateTo(ORIGINAL_ANGLE);
		} else {
			return;
		}
	}

	void baseLineAuto() {
		if (time_elapsed < 3.5) {
			myRobot.arcadeDrive(0.6, bearing);
		}
	}

	/**
	 * Use the x position of a rectangle to move in a straight line
	 */
	double curveToCenter(double pos) {
		if (Math.abs(pos - xRes / 2) > 20) {
			return -(pos - xRes / 2) / xRes;
		} else {
			return 0.0;
		}
	}

	/**
	 * The teleopInit method is called once each time the robot enters teleop
	 * mode
	 */
	public void teleopInit() {
		// Always reset Gyro in init methods. You don't necessarily need to
		// calibrate.
		gyro.calibrate();
		ORIGINAL_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGINAL_ANGLE);
		gyro.reset();
		// Starts with forward drive system (duh)
		isReversed = false;
	}

	/**
	 * This function is called periodically during operator control (teleOp
	 * mode)
	 */
	public void teleopPeriodic() {
		// Button 2 to reverse
		if (whiteR.getRawButton(7)) {
			isReversed = !isReversed;
		}
		// Assign Directional Magnitudes
		X = roundDown(whiteR.getX(), 0.007, 1);
		Y = roundDown(whiteR.getY(), 0.007, 1);
		Z = -roundDown(whiteR.getZ(), 0.1, 1);
		// Sets shooter power output; defaults to 0
		runShooter();
		// Use to get jammed balls out
		if (whiteL.getRawButton(8)) {
			shooter.set(-0.18);
		}
		// Pneumatics Controls
		if (whiteR.getTrigger()) {
			ds.set(DoubleSolenoid.Value.kReverse);
			SmartDashboard.putString("Hopper Door: ", "OPEN");
		} else {
			ds.set(DoubleSolenoid.Value.kForward);
			SmartDashboard.putString("Hopper Door: ", "CLOSED");
		}
		// Collector Belt Magnitude
		belt_speed = -slider(whiteL);
		collector.set(belt_speed);
		// Mechanum Drive. Default GYRO angle = 0.0
		if (isReversed) {
			X = -X;
			Y = -Y;
		}
		myRobot.mecanumDrive_Cartesian(X, Y, Z, bearing);
		// Publish values used in Teleop to SmartDashboard
		updateDashboard();
		updateDashboard(table);
	}

	/**
	 * Tries to rotate the robot to a desired bearing
	 */
	double rotateTo(double desiredAngle) {
		if (Math.abs((desiredAngle - gyro.getAngle()) / gyro.getAngle()) > 0.1) {
			return (Math.abs(desiredAngle - gyro.getAngle()) / 120.0) * (desiredAngle < gyro.getAngle() ? -1 : 1);
		} else {
			return 0;
		}
	}

	/**
	 * returns 0 if val is within the interval: [-limit ... limit] Otherwise
	 * returns val with outputs starting at limit.
	 */
	double roundDown(double val, double limit, double max) {
		limit = Math.abs(limit);
		max = Math.abs(max);
		if (limit > max) {
			limit = max;
		}
		if (Math.abs(val) > max) {
			val = max * (val < 0 ? -1 : 1);
		}
		return Math.abs(val) < limit ? 0 : (Math.abs(val) - limit) / (max - limit) * (val > 0 ? 1 : -1);
	}

	/**
	 * returns a Joystick's slider values from 0.0 to +1.0
	 */
	double slider(Joystick j) {
		return (j.getThrottle() + 1.0) / 2.0;
	}

	/**
	 * Runs shooter from 0 to 1
	 */
	private void runShooter() {
		shooter_speed = 0;
		// TODO:asdf
		if (whiteR.getRawButton(12)) {
			shooter_speed = .25;
		} else if (whiteR.getRawButton(10)) {
			shooter_speed = .755;
		} else if (whiteR.getRawButton(2)) {
			shooter_speed = roundDown(slider(whiteR), 0.1, 1.0);
		}

		shooter.set(shooter_speed);
	}

	/**
	 * Stops driving motors for 1 second
	 */
	public void stopDrive() {
		leftB.set(0);
		leftF.set(0);
		rightB.set(0);
		rightF.set(0);
		Timer.delay(1.0);
	}

	/**
	 * Slowly ramps the previous value to match current value by adding change
	 */
	double rampingValue(double currentValue, double previousValue, double change) {
		if (currentValue - previousValue > change)
			currentValue = previousValue + change;
		if (previousValue - currentValue > change)
			currentValue = previousValue - change;
		return currentValue;
	}

	void updateDashboard() {
		// SmartDashboard.putNumber("X-Magnitude", X);
		// SmartDashboard.putNumber("Y-Magnitude", Y);
		// SmartDashboard.putNumber("Z-Rotation", Z);
		SmartDashboard.putNumber("Bearing (Raw Gyro Angle)", bearing / GYRO_CONST);
		SmartDashboard.putNumber("Gyro Angle", bearing);
		SmartDashboard.putNumber("UltraSonic (Inches)", distance);
		SmartDashboard.putNumber("Shooter Magnitude", shooter_speed);
		SmartDashboard.putNumber("Belt Speed", belt_speed);
		SmartDashboard.putBoolean("Compressor Pressure Switch On", compressor.getPressureSwitchValue());
		SmartDashboard.putNumber("Compressor Current", compressor.getCompressorCurrent());
	}

	/**
	 * Update readings from Gyro and Ultrasonic
	 */
	void sensors() {
		// Get raw Gyro Angle
		bearing = gyro.getAngle();
		// Multiply gyro's angle by GRYO_CONST. Increasing GYRO_CONST will make
		// more rapid adjustments
		bearing *= GYRO_CONST;
		// Get Distance From UltraSonic (in Inches)
		distance = uss.getRangeInches();
	}

	/**
	 * Gives time elapsed in seconds between two times in milliseconds
	 */
	double time(double start, double finish) {
		return (finish - start) / 1000;
	}

	/**
	 * Publish Data Object to SmartDashboard
	 */
	void updateDashboard(Data... datas) {
		for (Data d : datas) {
			SmartDashboard.putString(d.getName(), d.getData());
		}
	}

	void updateDashboard(String name, double value) {
		SmartDashboard.putNumber(name, value);
	}

	/**
	 * Publishes all keys in a Network Table to SmartDashboard
	 */
	void updateDashboard(NetworkTable... tables) {
		double[] values;
		for (NetworkTable t : tables) {
			// System.out.println(t.getKeys());
			for (String k : t.getKeys()) {
				values = t.getNumberArray(k, new double[0]);
				SmartDashboard.putString(k, Arrays.toString(values).substring(1, Arrays.toString(values).length() - 1));
			}
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

}

enum SideGearAuto {
	OFF_CENTER(-1), DRIVE_STRAIGHT(0), ROTATE_LEFT(1), ROTATE_RIGHT(2), FAR_APPROACH(3), CLOSE_APPROACH(5), STOP(5);

	private int val;

	public int getValue() {
		return val;
	}

	private SideGearAuto(int v) {
		val = v;
	}
}

enum CenterGearAuto {
	FAR_APPROACH, OFF_CENTER, CLOSE_APPROACH, RESET_ANGLE, STOP;
}

enum LowBoilerAuto {

}