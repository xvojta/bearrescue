package stormbots;

import java.io.IOException;
import java.util.Arrays;

import lejos.hardware.device.UART;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;

public class Lidar extends Thread {

	private Port port;
	private UART uart;
	private java.io.InputStream uartIS;
	private byte[] lidarPacket;
	private volatile int actualDistance;
	private volatile int actualReflectivity;
	private volatile boolean doStop;

	public Lidar(Port port) {
		lidarPacket = new byte[9];
		this.setPort(port);
		uart = new UART(port);
		uart.setBitRate(115200);
		uartIS = uart.getInputStream();
		setActualDistance(-1);
		setActualReflectivity(-1);
		doStop = false;

	}

	private boolean isLidarPacketValid(byte[]lp){
		int checksum = 0;
		for (int i=0; i<lp.length-1; i++) {
			checksum += ((int) lidarPacket[i]) & 0xFF;
		}
		
		return lp[0]==0x59 && lp[1]==0x59 && (lp[8] == (byte)(checksum & 0xFF));
	}
	
	public void run() {
		while (!doStop) {
			Arrays.fill(lidarPacket, (byte) 0);
			while (lidarPacket[0] != 0x59)
				try {
					uartIS.read(lidarPacket, 0, 1);
				} catch (IOException e1) {
					e1.printStackTrace();
				}
			// synchronizace na zacatek paketu posun o byte at se najdeme
			int cnt;
			try {
				cnt = 1 + uartIS.read(lidarPacket, 1, 8);
				if (cnt == 9 && isLidarPacketValid(lidarPacket)){//lidarPacket[0] == 0x59 && lidarPacket[1] == 0x59) {
					setActualDistance((int) ((lidarPacket[3] << 8) & 0xFF00 | lidarPacket[2] & 0xFF));
					setActualReflectivity((int) ((lidarPacket[5] << 8) & 0xFF00 | lidarPacket[4] & 0xFF));
				}
			} catch (IOException e) {
				e.printStackTrace();

			}
		}
	}

	public void close() throws IOException, InterruptedException {
		doStop = true;
		Thread.sleep(1000);
		uartIS.close();
		uart.close();
	}

	public int getActualDistance() {
		return actualDistance;
	}

	public int getActualReflectivity() {
		return actualReflectivity;
	}

	public void setActualDistance(int actualDistance) {
		this.actualDistance = actualDistance;
	}

	public void setActualReflectivity(int actualreflectivity) {
		this.actualReflectivity = actualreflectivity;
	}

	public Port getPort() {
		return port;
	}

	public void setPort(Port port) {
		this.port = port;
	}
}
