package com.example.demo;

import java.awt.*;
import javax.swing.JFrame;

public class trialRobot extends Canvas {
    public void paint(Graphics g) {
        // Body
        g.setColor(Color.GRAY);
        g.fillRect(100, 100, 200, 300);

        // Wheels Front Left
        g.setColor(Color.black);
        g.fillRect(75, 150, 25, 50);

        // Wheels Front Right
        g.setColor(Color.black);
        g.fillRect(300, 150, 25, 50);

        // Wheels Back Left
        g.setColor(Color.black);
        g.fillRect(75, 300, 25, 50);

        // Wheels Back Right
        g.setColor(Color.black);
        g.fillRect(300, 300, 25, 50);
        
        // Front Sensor
        g.setColor(Color.yellow);
        g.fillRect(175, 100, 50, 50);

        // Camera
        g.setColor(Color.yellow);
        g.fillRect(250, 275, 50, 50);
        
        // Arms
        // g.setColor(Color.GRAY);
        // g.fillRect(50, 150, 75, 200);
        // g.fillRect(275, 150, 75, 200);
        
        // Legs
        // g.fillRect(125, 400, 50, 200);
        // g.fillRect(225, 400, 50, 200);
      }
      
    //   public static void main(String[] args) {
    //     JFrame frame = new JFrame("Robot Example");
    //     frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    //     frame.setSize(400, 600);
    //     frame.add(new trialRobot());
    //     frame.setVisible(true);
    //   }
}
