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
	final int[] buttonConstantsR = {//better if you make this a dictionary and use string names instead of indexes for when you reference the numbers
	//	values	indexes		description
		0,// 	- 			trigger for opening door
		0,//	1	
		2,//	2			slider shooter
		0,//	3
		0,//	4
		0,//	5
		0,//	6
		7,//	7			reverse direction
		8,//	8			reverse shooter
		9,//	9			activate movement with trigger
		10,//	10			high shooter
		0,//	11	
		12,//	12			low shooter
		
		
};
	final int[] buttonConstantsL = {
		//slider for collector
		
		
};
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	

	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	final String choicesAuto = "Auto choices";
	// Robot Drive Variable
	private RobotDrive myRobot;
	// Motor Controllers
	private TalonSRX rightB, leftB, rightF, leftF, collector, shooter;
	// Servo Motor
//	private Servo servo;
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
	DoubleSolenoid shooterClose;
	DoubleSolenoid agitator;
	Compressor compressor;
	// Joysticks
	private Joystick whiteR, whiteL;
	// Network Table
	private NetworkTable table;
	private double SHOOTER_SPEED, BELT_SPEED, GYRO_ANGLE;
	private double X, Y, Z;
	private boolean isReversed;
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		table = NetworkTable.getTable("GRIP/targets");
		NetworkTable.setTeam(835);
		
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData(choicesAuto, chooser);
		autoSelected = chooser.getSelected();

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
		shooterClose = new DoubleSolenoid(0, 1);
		agitator = new DoubleSolenoid(2, 3);
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);

		// Instantiate Ultrasonic Sensor. The first parameter is the analog Ping
		// Channel. The second parameter is the Analog Echo Channel
		 uss = new Ultrasonic(0, 1);
		 uss.setAutomaticMode(true);

		// Instantiate Joysticks
		whiteR = new Joystick(0);
		whiteL = new Joystick(1);

		// Instantiate RobotDrive with 4 Motor Controllers
		myRobot = new RobotDrive(leftF, leftB, rightF, rightB);		
	}

	int loopTimer;

	/**
	 * The autonomousInit method is called once each time the robot enters
	 * autonomous mode
	 */
	
	public void autonomousInit() {

		autoSelected = table.getString(choicesAuto, defaultAuto);
		
		gyro.reset();

		ORIGIN_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGIN_ANGLE);

		// Starts with drive system forwards, towards gear collector
		isReversed = false;
	}

	public void autonomousPeriodic() {
		
		double power=0, curve;
		updateDashboard("Ultrasonic", uss.getRangeInches());
		if(uss.getRangeInches() <= 12)
			power = .3;
		
		// power = 0;
		 curve = 0;
		myRobot.arcadeDrive(power, curve);
		System.out.println(""+ power+":"+ curve);
	}
	
	
	


	/**
	 * The teleopInit method is called once each time the robot enters teleop
	 * mode
	 */
	public void teleopInit() {
		// Always reset Gyro in init methods. You don't necessarily need to
		// calibrate.
		gyro.calibrate();
		gyro.reset();
		ORIGIN_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGIN_ANGLE);

		// Starts with forward drive system (duh)
		isReversed = false;
	}

	/**
	 * This function is called periodically during operator control (teleOp
	 * mode)
	 */
	public void teleopPeriodic() {
		System.out.println("tele");
		updateDashboard("UltraSonic", uss.getRangeInches());
		// Button 7 to reverse
		if (whiteR.getRawButton(buttonConstantsR[7])) {
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
		

		// Pneumatics Controls
		if (whiteR.getTrigger()) {
			shooterClose.set(DoubleSolenoid.Value.kReverse);
			SmartDashboard.putString("Hopper Door: ", "OPEN");
		} else {
			shooterClose.set(DoubleSolenoid.Value.kForward);
			SmartDashboard.putString("Hopper Door: ", "CLOSED");
		}
		
		if (whiteR.getRawButton(5)) {
			agitator.set(DoubleSolenoid.Value.kReverse);
			SmartDashboard.putString("Agitator: ", "OPEN");
		} else if (whiteR.getRawButton(3)) {
			agitator.set(DoubleSolenoid.Value.kForward);
			SmartDashboard.putString("Agitator: ", "CLOSED");
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
		//TODO:change controls to something convenient
		if (whiteR.getRawButton(buttonConstantsR[8])) {
			shooter.set(-0.18);
		}
		else if (whiteR.getRawButton(buttonConstantsR[12]))
		{
			SHOOTER_SPEED = .25;
		}
		else if (whiteR.getRawButton(buttonConstantsR[10])) {
			SHOOTER_SPEED = .755;
		}
		else if (whiteR.getRawButton(buttonConstantsR[2])) {
			
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
		SmartDashboard.putBoolean("Compressor Pressure Switch On: ", compressor.getPressureSwitchValue());
		SmartDashboard.putNumber("Compressor Current: ", compressor.getCompressorCurrent());
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

	void steerForwardsGear() {
		myRobot.mecanumDrive_Cartesian(0, getPowerGear(), getRotationGear(), gyro.getAngle());
	}
	

	void leftLiftAutonomous() {
		int step = 0;
		long start = System.nanoTime();
//		final double[] steps = {
//				start,
//				3 * Math.pow(10, 9),
//				5 * Math.pow(10, 9),
//				7 * Math.pow(10, 9),
//				15 * Math.pow(10, 9),
//				200
//		}; 
//			
//		if(System.nanoTime() - steps[stepCount] < steps[stepCount +1]) {
//			stepCount++;
//		}
		
		switch(step){ 
			case 0:
				myRobot.mecanumDrive_Polar(.7, 0, 0);//TODO: find time
				if(System.nanoTime() - start >= 3 * Math.pow(10, 9)) return;
				step++;
				//DRIVE forwards
				break;
			case 1:
				if(getRotationGear() <= 20 && getRotationGear() >= -20) {
					step++;
				}
				else myRobot.mecanumDrive_Polar(0, 0, .5);//turn and locate
				break;
			case 2:
				if(getDistanceGearInches() != 0)
					steerForwardsGear();
				else step++;
				//fowards and look for tape
				break;
//			case 3:
//				if(getDistanceGearInches() <= 10)
//					myRobot.mecanumDrive_Cartesian(0, -.5, getRotationGear(), gyro.getAngle());
//				else step++;
//				//backwards 
//				break;
			case 4:
				
				break;
			case 5:
				
				break;
				//go for ending
			default:
				myRobot.mecanumDrive_Polar(0, 0, 0);
				return;
		}
		
	}
	double getDistanceGearInches() {
		double distance = uss.getRangeInches();
//		double[] heights;
//		double distance;
//		heights = table.getNumberArray("height", new double[0]);
//
//		if (heights.length != 0) {
//			distance = 5 / 12 * yRes / (2 * heights[0] * Math.tan(54 * Math.PI / 180));
//		} else {
//			distance = 0;
//		}
		return distance;
	}
	double getPowerGear() {
		double distance, power;
		distance = getDistanceGearInches();

		if (distance > 0.5) {
			power = 5.0 / 9;
		} else {
			power = 0;
		}
		SmartDashboard.putNumber("distance", distance);

		SmartDashboard.putNumber("power", power);
		return power;
	}
	double getRotationGear() {
		double x, curve = 0;
		double[] xvalues, areas;
		areas = table.getNumberArray("area", new double[0]);
		if (areas.length == 0) {
			return .7;
		}
		xvalues = table.getNumberArray("centerY", new double[0]);//accounts for camera rotation
		if (xvalues.length < 2) {
			x = xRes / 2;
		} else {
			x = xvalues[0] + xvalues[1];
			x /= 2;
		}
		curve = cameraXtoRotation(x);
		SmartDashboard.putNumber("CenterX", x);
		SmartDashboard.putNumber("PostCurve", curve);
		return curve;
	}
	double cameraXtoRotation(double x) {
		return -(x - xRes / 2) / xRes;
	}
	double roundToHalf(double value) {
		value *= Math.sqrt(2.0);
		if (value > 0.5) {
			return 0.5;
		}
		if (value < -0.5) {
			return -0.5;
		}
		else return 0;
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
