package org.usfirst.frc157.Tekeva2018.subsystems;

import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JFrame;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class PathManager extends JFrame implements Sendable {
	
	private double startTime;
	private double curX = 0;
	private double curY = 0;
	private double oldX = 0;
	private double oldY = 0;
	private double oldEncoder = 0;
	private double oldAngle = 0;
	private ArrayList<Point> points = new ArrayList<Point>();
	private String m_name = "";
	private String m_subsystem = "Ungrouped";
	
	public PathManager() {
		super("Path Manager");
		setSize(800,800);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		setResizable(false);
		setVisible(true);
		points.add(new Point(0,0));
		LiveWindow.add(this);
       // startTime = Timer.getFPGATimestamp();
	}
	
	public void update(double encoder, double angle) {
		angle = -angle;
		double deltaEnc = encoder - oldEncoder;
		curX = deltaEnc*Math.cos(Math.toRadians(angle)) + oldX;
		curY = deltaEnc*Math.sin(Math.toRadians(angle)) + oldY;
		System.out.println(curX + ", " +curY);
		points.add(new Point(curX, curY));
		oldX = curX;
		oldY = curY;
		repaint();
	}
	
	public void paint(Graphics g) {
		for (Point point: points) {
			g.fillOval((int)point.getX()+400, (int)point.getY()+400, 5, 5);
		}
	}

	@Override
	public String getSubsystem() {
		// TODO Auto-generated method stub
		return m_subsystem;
	}

	@Override
	public void setSubsystem(String subsystem) {
		// TODO Auto-generated method stub
		 m_subsystem = subsystem;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
		
	}
}
