package br.ufal.robotic.linefollowing;

import org.lejos.ev3.sample.sensorfilter.AutoAdjustFilter;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Key;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class LineFolling {

	private boolean isHolonomic = false;

	public static void main(String[] args) {
		new LineFolling();

		System.exit(0);
	}

	public LineFolling() {

		Chassis chassis;

		if (isHolonomic) {
			/*********************************
			 * Holonomics Wheels
			 *********************************/
			double radius = 135;
			double diameterD = 48;
			Wheel wheel1 = WheeledChassis.modelHolonomicWheel(Motor.A, diameterD).polarPosition(0, radius).gearRatio(2);
			Wheel wheel2 = WheeledChassis.modelHolonomicWheel(Motor.B, diameterD).polarPosition(120, radius)
					.gearRatio(2);
			Wheel wheel3 = WheeledChassis.modelHolonomicWheel(Motor.C, diameterD).polarPosition(240, radius)
					.gearRatio(2);
			chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2, wheel3 }, WheeledChassis.TYPE_HOLONOMIC);
		} else {
			/*********************************
			 * Diffentials Wheels
			 *********************************/
			double offset = 70;
			double diameterH = 48;
			Wheel wheelA = WheeledChassis.modelWheel(Motor.B, diameterH).offset(-offset);
			Wheel wheelB = WheeledChassis.modelWheel(Motor.C, diameterH).offset(offset);
			chassis = new WheeledChassis(new Wheel[] { wheelA, wheelB }, WheeledChassis.TYPE_DIFFERENTIAL);
		}

		MovePilot pilot = new MovePilot(chassis);
		pilot.setLinearSpeed(100); // 10cm/s

		/******************************* Sensor *****/
		Brick brick = BrickFinder.getDefault();
		Port port = brick.getPort("S4");

		EV3ColorSensor sensor = new EV3ColorSensor(port);

		SampleProvider redMode = sensor.getRedMode();

		SampleProvider reflectedMode = new AutoAdjustFilter(redMode); // calibration

		int sampleSize = reflectedMode.sampleSize();
		float[] samples = new float[sampleSize];

		Key escape = brick.getKey("Escape");
		while (!escape.isDown()) {
			pilot.forward();
			//Thread.yield();
			reflectedMode.fetchSample(samples, 0);

			if (samples[0] > 0.5) {
				// turn to right
				pilot.rotate(45);
			} else {
				// turn to left
				pilot.rotate(-45);
			}
		}

		sensor.close();

	}

}
