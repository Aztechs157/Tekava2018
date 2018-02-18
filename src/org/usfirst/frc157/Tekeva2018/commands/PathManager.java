package org.usfirst.frc157.Tekeva2018.commands;

import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JFrame;

public class PathManager extends JFrame {
	
	private double startTime;
	private double curX = 0;
	private double curY = 0;
	private double oldX = 0;
	private double oldY = 0;
	private double oldEncoder = 0;
	private double oldAngle = 0;
	private ArrayList<Point> points = new ArrayList<Point>();
	
	public PathManager() {
		super("Path Manager");
		setSize(800,800);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		setResizable(false);
		setVisible(true);
		points.add(new Point(0,0));
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
}
