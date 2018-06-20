package stormbots;

import java.io.IOException;
import java.util.Arrays;

import lejos.hardware.device.UART;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;

public class StopButton extends Thread {

	private Port port;

	public StopButton(Port port) {
		this.setPort(port);
	}
	
	public void run() {
		EV3TouchSensor touchSensor = new EV3TouchSensor(port);
		SensorMode touch = touchSensor.getTouchMode();
		float[] sample = new float[touch.sampleSize()];
		do{
			touch.fetchSample(sample, 0);
		} while (sample[0] == 0);
		
		System.exit(0);
	}

	public Port getPort() {
		return port;
	}

	public void setPort(Port port) {
		this.port = port;
	}
}
