package org.usfirst.frc.team835.robot;

public class Number extends Data{

	private double value;
	
	public Number(String s, double d){
		super(s);
		value = d;
	}
	
	public String getData(){
		return value + "";
	}
}
