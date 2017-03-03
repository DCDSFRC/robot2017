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
	// Servo Motor
	private Servo servo;
	// Camera Resolution
	private int xRes = 240;
	private int yRes = 360;
	// Gyroscope
	private ADXRS450_Gyro gyro;
	private final double GYRO_CONST = 0.15;
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
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	// Other variables
	private double shooter_speed, belt_speed, bearing, distance;
	private double X, Y, Z;
	private boolean isReversed;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// Retrieve NetworkTables
		table = NetworkTable.getTable("GRIP/targets");
		NetworkTable.setTeam(835);
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
		// Sendable Chooser
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("1. Gear Position Left", "leftG");
		chooser.addObject("2. Gear Position Center", "centerG");
		chooser.addObject("3. Gear Position Right", "rightG");
		SmartDashboard.putData("Auto choices", chooser);
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
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		SmartDashboard.putString("Autonomous Mode", autoSelected);
		// Pre-auto stuff
		gyro.calibrate();
		ORIGINAL_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGINAL_ANGLE);
		gyro.reset();
		// Starts with forward drive system (collector in front)
		isReversed = false;
	}

	public void autonomousPeriodic() {
		sensors();
		switch (autoSelected) {
		case "leftG":
			break;
		case "centerG":
			centerLiftAutonomous();
			break;
		case "rightG":
			break;
		default:
			break;
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
	 * Autonomous Routine for when the robot is positioned in the center
	 * position
	 */
	void centerLiftAutonomous() {
		// Step 0: Get Tape Contours's position from NetworkTable
		double[] centerX = table.getNumberArray("centerX", new double[0]);
		double x;
		if (centerX.length >= 2) {
			x = (centerX[0] + centerX[1]) / 2.0;
		} else {
			x = xRes / 2;
		}
		// Step 1: Approach the gear-lift at a high speed (50-80%)
		double curve;
		curve = curveToCenter(x);
		if (distance > 18) {
			myRobot.mecanumDrive_Cartesian(0, 0.8, curve, bearing);
		} else if (distance <= 18 && distance > 15) {
			myRobot.mecanumDrive_Cartesian(0, 0.3, curve / 2.0, bearing);
		} else if (distance <= 15) {
			// do nothing
			return;
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

		updateDashboard("UltraSonic", uss.getRangeInches());
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

	/*
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
		SmartDashboard.putNumber("X-Magnitude: ", X);
		SmartDashboard.putNumber("Y-Magnitude: ", Y);
		SmartDashboard.putNumber("Z-Rotation: ", Z);
		SmartDashboard.putNumber("Bearing (Raw Gyro Angle): ", bearing / GYRO_CONST);
		SmartDashboard.putNumber("Gyro Angle: ", bearing);
		SmartDashboard.putNumber("Shooter Magnitude: ", shooter_speed);
		SmartDashboard.putNumber("Belt Speed: ", belt_speed);
		// SmartDashboard.putBoolean("Compressor Pressure Switch On: ",
		// compressor.getPressureSwitchValue());
		// SmartDashboard.putNumber("Compressor Current: ",
		// compressor.getCompressorCurrent());
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
