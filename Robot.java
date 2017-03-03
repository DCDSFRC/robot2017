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
	private final double GYRO_CONST = 0.08;
	private double ORIGINAL_ANGLE;
	// Ultrasonic Sensor
	Ultrasonic uss;
	// Pneumatics REMOVED
	// DoubleSolenoid ds;
	// Compressor compressor;
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
	private double X, Y, Z;
	private boolean isReversed;
	AutoStep gStep;
	private double autoStartTime, autoAngleTarget;
	int loopTimer;
	private boolean autoControl;

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
		 winch = new TalonSRX(6);
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
		// Instantiate Pnuematics Components REMOVED
		// ds = new DoubleSolenoid(0, 1);
		// compressor = new Compressor(0);
		// compressor.setClosedLoopControl(true);
		// Instantiate Ultrasonic Sensor. The first parameter is the Ping
		// Channel. The second parameter is the Echo Channel. Both are on DIO
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
		SmartDashboard.putData("Autonomous choices", chooser);
	}

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
		autoStartTime = System.nanoTime();
	}

	public void autonomousPeriodic() {
		double angle = bearing();
		double d = distance();
		autoAngleTarget = 0;
		switch (autoModeSelected) {// Enums look cool
		case "centerG":
			if (gStep == AutoStep.STOP) {
				break;
			}
			if (currentAutoTime() < 3 || d > 24) {
				gStep = AutoStep.FAR_APPROACH;
			} else if (currentAutoTime() < 6 || d <= 24) {
				gStep = AutoStep.CLOSE_APPROACH;
			} else if (currentAutoTime() < 9 || d <= 14) {
				autoAngleTarget = 0;
				gStep = AutoStep.RESET;
			} else if (currentAutoTime() > 12 || d <= 11) {
				gStep = AutoStep.STOP;
			} else { // should never happen
				System.out.println("BIG ERROR");
			}
			auto();
//			centerGearAuto();
			break;
		case "leftG":
			if (gStep == AutoStep.STOP) {
				break;
			}
			if (currentAutoTime() < 1.0) {
				gStep = AutoStep.RUN_STRAIGHT;
			} else if (currentAutoTime() < 5 || !(angle < 75 && angle > 45)) {
				autoAngleTarget = 60;
				gStep = AutoStep.ROTATE_RIGHT;
			} else if ((currentAutoTime() < 8 && d > 24)) {
				gStep = AutoStep.FAR_APPROACH;
			} else if (currentAutoTime() < 12 && d > 15) {
				gStep = AutoStep.CLOSE_APPROACH;
			} else if (currentAutoTime() < 14 || d <= 15) {
				autoAngleTarget = 60;
				gStep = AutoStep.RESET;
			} else if (currentAutoTime() >= 14 || d <= 11) {
				gStep = AutoStep.STOP;
			} else { // should never happen
				System.out.println("BIG ERROR");
			}
			auto();
//			leftGearAuto(true);
			break;
		case "rightG":
			if (gStep == AutoStep.STOP) {
				break;
			}
			if (currentAutoTime() < 3) {
				gStep = AutoStep.RUN_STRAIGHT;
			} else if (currentAutoTime() < 5) {
				autoAngleTarget = -60;
				gStep = AutoStep.ROTATE_LEFT;
			} else if ((currentAutoTime() < 8) || !(angle > -75 && angle < -45)) {
				gStep = AutoStep.FAR_APPROACH;
			} else if (currentAutoTime() < 12 || d <= 24) {
				gStep = AutoStep.CLOSE_APPROACH;
			} else if (currentAutoTime() < 14 || d <= 14) {
				autoAngleTarget = -60;
				gStep = AutoStep.RESET;
			} else if (currentAutoTime() >= 14 || d <= 13) {
				gStep = AutoStep.STOP;
			} else { // should never happen
				System.out.println("BIG ERROR");
			}
			auto();
//			leftGearAuto(false);
			break;
		default:
			if (currentAutoTime() < 5.5) {
				gStep = AutoStep.RUN_STRAIGHT;
			} else {
				gStep = AutoStep.STOP;
			}
			auto();
//			baseLineAuto();
			break;
		}
		loopTimer++;
	}

	/**
	 * Autonomous Routine for putting gear in side peg. Parameter onLeft used to
	 * determine which way robot should turn while approaching
	 */
	void auto() {
		double x = xRes;
		double curve = 0;
		if (gStep.usesVision()) {
			double[] centerX = table.getNumberArray("centerX", new double[0]);
			if (centerX.length == 1) {
				x = centerX[0];
			} else if (centerX.length >= 2) {
				x = (centerX[0] + centerX[1]) / 2.0;
			} else {
				x = xRes / 2;
			}
			curve = -curveToCenter(x);
		}
		X = 0;
		Y = 0;
		Z = curve;
		double angle = bearing();
		if (x < 0.4 * xRes || x > .6 * xRes) {
			gStep = AutoStep.RE_CENTER;
		}
		if (gStep == AutoStep.RE_CENTER) {
			X = (x - xRes / 2) / xRes;
			Z = curve;
		} else if (gStep == AutoStep.RUN_STRAIGHT) {
			Y = -0.75;
		} else if (gStep == AutoStep.ROTATE_LEFT) {
			Z = angleRotation(autoAngleTarget);
		} else if (gStep == AutoStep.ROTATE_RIGHT) {
			Z = angleRotation(autoAngleTarget);
		} else if (gStep == AutoStep.FAR_APPROACH) {
			Y = -0.5;
			Z = curve;
			angle *= 1.25;
		} else if (gStep == AutoStep.CLOSE_APPROACH) {
			Y = -0.25;
			Z = curve;
		} else if (gStep == AutoStep.STOP) {
			Z = 0;
		} else if (gStep == AutoStep.RESET) {
			Y = -0.15;
			Z = angleRotation(autoAngleTarget) * 2;
		}
		myRobot.mecanumDrive_Cartesian(X, Y, Z, angle);
	}

	/**
	 * Autonomous Routine for putting gear in center peg
	 */
	void centerGearAuto() {
		double[] centerX = table.getNumberArray("centerY", new double[0]);
		double x;
		if (centerX.length >= 2) {
			x = (centerX[0] + centerX[1]) / 2.0;
		} else if (centerX.length == 1) {
			x = centerX[0];
		} else {
			x = xRes / 2;
		}
		double curve = -curveToCenter(x);
		double d = distance();
		double angle = bearing();

		if (d >= 50) {
			myRobot.mecanumDrive_Cartesian(0, -0.6, curve, angle);
		} else if (d >= 18) {
			if (x < .4 * xRes) {
				myRobot.mecanumDrive_Cartesian(-0.65, 0, curve, angle);
			} else if (x > 0.6 * xRes) {
				myRobot.mecanumDrive_Cartesian(0.65, 0, curve, angle);
			} else {
				myRobot.mecanumDrive_Cartesian(0, -0.5, curve, angle);
			}
		} else if (d >= 15) {
			myRobot.mecanumDrive_Cartesian(0, -0.3, curve, angle);
		} else if (d < 11.5) {
			angleRotation(ORIGINAL_ANGLE);
		}
	}
	
	void leftGearAuto(boolean RobotOnLeft) {
		double x = xRes;
		double curve = 0;
		if (gStep.usesVision()) {
			double[] centerX = table.getNumberArray("centerX", new double[0]);
			if (centerX.length >= 2) {
				x = (centerX[0] + centerX[1]) / 2.0;
			} else {
				x = xRes / 2;
			}
			curve = curveToCenter(x);
		}
		double angle = bearing();
		if (x < 0.4 * xRes || x > .6 * xRes) {
			gStep = AutoStep.RE_CENTER;
		}
		X = 0;
		Y = 0;
		Z = curve;

		if (gStep == AutoStep.RE_CENTER) {
			X = (x - xRes / 2) / xRes;
		} else if (gStep == AutoStep.RUN_STRAIGHT) {
			Y = 0.9;
			angle *= 1.5;
		} else if (gStep == AutoStep.ROTATE_LEFT) {
			Z = angleRotation(-60);
		} else if (gStep == AutoStep.ROTATE_RIGHT) {
			Z = angleRotation(60);
		} else if (gStep == AutoStep.FAR_APPROACH) {
			Y = 0.6;
			angle *= 1.2;
		} else if (gStep == AutoStep.CLOSE_APPROACH) {
			X = 0;
			Y = 0.3;
		} else if (gStep == AutoStep.STOP) {
			angle = 0;
		}
		myRobot.mecanumDrive_Cartesian(X, Y, Z, angle);
	}

	void baseLineAuto() {
		if (currentAutoTime() < 5.5) {
			myRobot.arcadeDrive(0.65, bearing());
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
		// Starts with forward drive system (collector in front)
		isReversed = false;
		autoControl = false;
	}

	/**
	 * This function is called periodically during operator control (teleOp
	 * mode)
	 */
	public void teleopPeriodic() {
		if (autoControl == true) {
			autonomousPeriodic();

			return;
		}
		if(whiteR.getTrigger()){
			winch.set(whiteR.getThrottle());
		}
		// Button to reverse
		if (whiteR.getRawButton(7)) {
			isReversed = !isReversed;
		}
		// Assign Directional Magnitudes
		X = roundDown(whiteR.getX(), 0.1, 1);
		Y = roundDown(whiteR.getY(), 0.1, 1);
		Z = -roundDown(whiteR.getZ(), 0.2, 1);

		// Mechanum Drive. Default GYRO angle = 0.0
		if (isReversed) {
			X = -X;
			Y = -Y;
		}
		double angle = bearing();
		myRobot.mecanumDrive_Cartesian(X, Y, Z, angle);
		// Publish values used in Teleop to SmartDashboard
		updateDashboard();
		updateDashboard(table);
	}

	/**
	 * Tries to rotate the robot to a desired bearing
	 */
	double angleRotation(double desiredAngle) {
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
		SmartDashboard.putNumber("X-Magnitude", X);
		SmartDashboard.putNumber("Y-Magnitude", Y);
		SmartDashboard.putNumber("Z-Rotation", Z);
		// SmartDashboard.putNumber("Bearing (Raw Gyro Angle)", bearing /
		// GYRO_CONST);
		// SmartDashboard.putNumber("Gyro Angle", bearing);
		// SmartDashboard.putNumber("UltraSonic (Inches)", distance);
		// SmartDashboard.putNumber("Shooter Magnitude", shooter_speed);
		// SmartDashboard.putNumber("Belt Speed", belt_speed);
		// SmartDashboard.putBoolean("Compressor Pressure Switch On",
		// compressor.getPressureSwitchValue());
		// SmartDashboard.putNumber("Compressor Current",
		// compressor.getCompressorCurrent());
	}

	double distance() {
		double rangeCompensation = 9;
		SmartDashboard.putNumber("Distance", uss.getRangeInches());
		return (uss.getRangeInches() - rangeCompensation < 0 ? 0 : uss.getRangeInches() - rangeCompensation);
	}

	double bearing() {
		SmartDashboard.putNumber("Bearing", gyro.getAngle());
		return gyro.getAngle();
	}

	/**
	 * Gives time elapsed in seconds between two times in milliseconds
	 */

	double currentAutoTime() {
		return ((double) System.nanoTime() - autoStartTime) / Math.pow(10, 9);
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

	public void testInit() {
		// Always reset Gyro in init methods. You don't necessarily need to
		// calibrate.
		gyro.calibrate();
		gyro.reset();
		ORIGINAL_ANGLE = gyro.getAngle();
		SmartDashboard.putNumber("Original Gyro Angle", ORIGINAL_ANGLE);
		// Starts with forward drive system (duh)
		isReversed = false;
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {

		myRobot.mecanumDrive_Cartesian(0, 0, angleRotation(90), bearing());
		LiveWindow.run();
	}

}

enum AutoStep {
	RE_CENTER(true), RUN_STRAIGHT(false), ROTATE_LEFT(false), ROTATE_RIGHT(false), FAR_APPROACH(true), CLOSE_APPROACH(
			true), STOP(false), RESET(false);

	private boolean vision;

	public boolean usesVision() {
		return vision;
	}

	private AutoStep(boolean v) {
		vision = v;
	}
}

enum JoysticButtons {
	// TODO: Map joystick to enum
}