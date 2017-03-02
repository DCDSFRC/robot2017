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
	private double ORIGIN_ANGLE;
	// Ultrasonic Sensor
	Ultrasonic uss;
	// Pneumatics
	DoubleSolenoid ds;
	Compressor compressor;
	// Joysticks
	private Joystick whiteR, whiteL;
	// Network Table
	private NetworkTable table;
	private double SHOOTER_SPEED, BELT_SPEED, GYRO_ANGLE;
	private double X, Y, Z;
	private boolean isReversed;
	double x, distance, power, curve = 0;
	double[] xvalues, areas, heights, widths;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
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
		ORIGIN_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGIN_ANGLE);

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
	}

	int loopTimer;

	/**
	 * The autonomousInit method is called once each time the robot enters
	 * autonomous mode
	 */
	
	public void autonomousInit() {
		gyro.calibrate();
		ORIGIN_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGIN_ANGLE);
		gyro.reset();

		// Starts with forward drive system (duh)
		isReversed = false;
	}

	public void autonomousPeriodic() {
		updateDashboard("Ultrasonic", uss.getRangeInches());

		
//		setTables(table);
		areas = table.getNumberArray("area", new double[0]);
		if (areas.length == 0) {
			myRobot.arcadeDrive(0, 0.7);
			return;
		}
		heights = table.getNumberArray("height", new double[0]);
		xvalues = table.getNumberArray("centerX", new double[0]);
		if (xvalues.length == 0) {
			x = xRes / 2;
		} else {
			x = xvalues[0];
		}
		curve = curveToCenter(x);
		SmartDashboard.putNumber("X", x);
		SmartDashboard.putNumber("PreCurve", curve);
		curve *= -Math.sqrt(2.0);
		if (curve > 0.5) {
			curve = 0.5;
		}
		if (curve < -0.5) {
			curve = -0.5;
		}
		if (heights.length != 0) {
			distance = 5 / 12 * yRes / (2 * heights[0] * Math.tan(54 * Math.PI / 180));
		} else {
			distance = 0;
		}

		if (distance > 0.5) {
			power = 5.0 / 9;
		} else {
			power = 0;
		}
		SmartDashboard.putNumber("PostCurve", curve);
		SmartDashboard.putNumber("distance", distance);
		SmartDashboard.putNumber("power", power);
		// power = 0;
		// curve = 0;
		myRobot.arcadeDrive(power, curve);
	}
	

	


	/**
	 * The teleopInit method is called once each time the robot enters teleop
	 * mode
	 */
	public void teleopInit() {
		// Always reset Gyro in init methods. You don't necessarily need to
		// calibrate.
		gyro.calibrate();
		ORIGIN_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGIN_ANGLE);
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

		// Run Gyro
		gyroscope();

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
		BELT_SPEED = -slider(whiteL);
		collector.set(BELT_SPEED);

		// Mechanum Drive. Default GYRO angle = 0.0
		if (isReversed) {
			X = -X;
			Y = -Y;
		}
		myRobot.mecanumDrive_Cartesian(X, Y, Z, GYRO_ANGLE);
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
		SHOOTER_SPEED = 0;
		//TODO:asdf
		if (whiteR.getRawButton(12))
		{
			SHOOTER_SPEED = .25;
		}
		else if (whiteR.getRawButton(10)) {
			SHOOTER_SPEED = .755;
		}
		else if (whiteR.getRawButton(2)) {
			
			SHOOTER_SPEED = roundDown(slider(whiteR), 0.1, 1.0);
		}
		
		shooter.set(SHOOTER_SPEED);
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

	void updateDashboard(){
		SmartDashboard.putNumber("X-Magnitude: ", X);
		SmartDashboard.putNumber("Y-Magnitude: ", Y);
		SmartDashboard.putNumber("Z-Rotation: ", Z);
		SmartDashboard.putNumber("Bearing (Raw Gyro Angle): ", GYRO_ANGLE / GYRO_CONST);
		SmartDashboard.putNumber("Gyro Angle: ", GYRO_ANGLE);
		SmartDashboard.putNumber("Shooter Magnitude: ", SHOOTER_SPEED);
		SmartDashboard.putNumber("Belt Speed: ", BELT_SPEED);
//		SmartDashboard.putBoolean("Compressor Pressure Switch On: ", compressor.getPressureSwitchValue());
//		SmartDashboard.putNumber("Compressor Current: ", compressor.getCompressorCurrent());
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

	void goForward(double power) {
		//Step 1: TODO: go forward past the airship
		myRobot.mecanumDrive_Cartesian(0, power, 0, gyro.getAngle());
	}

	void steer() {
		//Step 2: TODO: Rotate to a bearing facing the airship
		if(GYRO_ANGLE < 200 || GYRO_ANGLE > 270){
			myRobot.mecanumDrive_Cartesian(0, 0, .4, 0);
		}
	}

	void steerForward() {
		//Step 3: TODO: Use UltraSonic to get distance to the airship
		int xPos = 0;
		myRobot.mecanumDrive_Cartesian(0, .35, curveToCenter(xPos), GYRO_ANGLE);
	}

	double curveToCenter(double pos) {
		if (Math.abs(pos - xRes / 2) > 20) {
			return (pos - xRes / 2) / xRes;
		} else {
			return 0.0;
		}
	}

	void gyroscope() {
		// Get raw Gyro Angle
		GYRO_ANGLE = gyro.getAngle();

		// Multiply gyro's angle by GRYO_CONST. Increasing GYRO_CONST will make
		// more rapid adjustments
		GYRO_ANGLE *= GYRO_CONST;
	}

	
	
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

}
