package br.ufal.robotic.square;

import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class Square {

	private boolean isHolonomic = false;
	public static void main(String[] args) {
		new Square();
	    
	    System.exit(0);
	}
	public Square(){
		double distance = 450; //45cm
		double angle = 90;
		Chassis chassis;	    
	    
	    if(isHolonomic){
	    	/********************************* Holonomics Wheels *****************************************/
			double radius = 135;
			double diameterD = 48;
			Wheel wheel1 = WheeledChassis.modelHolonomicWheel(Motor.A, diameterD).polarPosition(0, radius).gearRatio(2);
		    Wheel wheel2 = WheeledChassis.modelHolonomicWheel(Motor.B, diameterD).polarPosition(120, radius).gearRatio(2);
		    Wheel wheel3 = WheeledChassis.modelHolonomicWheel(Motor.C, diameterD).polarPosition(240, radius).gearRatio(2);
		    chassis = new WheeledChassis(new Wheel[]{wheel1, wheel2, wheel3}, WheeledChassis.TYPE_HOLONOMIC);
	    }
	    else{
	    	/********************************* Diffentials Wheels*****************************************/
			double offset = 70;
			double diameterH = 48;
		    Wheel wheelA = WheeledChassis.modelWheel(Motor.B, diameterH).offset(-offset);
		    Wheel wheelB = WheeledChassis.modelWheel(Motor.C, diameterH).offset(offset);
		    chassis = new WheeledChassis(new Wheel[]{wheelA,wheelB}, WheeledChassis.TYPE_DIFFERENTIAL);
	    }
	    
	    MovePilot pilot = new MovePilot(chassis);
	    pilot.setLinearSpeed(100); //10cm/s
	    pilot.travel(distance);
	    pilot.rotate(angle);
	    
	    pilot.travel(distance);
	    pilot.rotate(angle);
	    
	    pilot.travel(distance);
	    pilot.rotate(angle);
	    
	    pilot.travel(distance);
	    pilot.rotate(angle);
	    
	    pilot.stop();
	}
}
