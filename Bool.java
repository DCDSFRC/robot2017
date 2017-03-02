package org.usfirst.frc.team835.robot;

public class Bool extends Data{
	boolean bool;
	
	public Bool(String s, boolean b){
		super(s);
		bool = b;
	}
	
	public String getData(){
		return bool + "";
	}
}
