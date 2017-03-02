package org.usfirst.frc.team835.robot;

public abstract class Data {
	private String name;

	public Data(String s){
		name = s;
	}
	
	public String getName(){
		return name;
	}
	public abstract String getData();
}
