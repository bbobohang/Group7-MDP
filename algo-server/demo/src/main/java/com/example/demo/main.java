package com.example.demo;

import javax.swing.*;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.springframework.boot.jackson.JsonObjectDeserializer;


import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.Point;

public class main extends JPanel {

	private final int width = 500, height = 500;
	private final int cellWidth = 25, cellHeight = 25;

	Map<Point, Color> tiles = new HashMap<Point, Color>();

	Node start = new Node(new Point(25, 450), cellWidth, cellHeight, Direction.UP);

	Node robot = new Node(new Point(25, 450), cellWidth, cellHeight, Direction.UP);

	LinkedHashMap<String, Node> obstacles = new LinkedHashMap<>();
	LinkedHashMap<String, Node> goals = new LinkedHashMap<>();
	Set<Node> open = new HashSet<Node>();
	Set<Node> closed = new HashSet<Node>();
	
	Thread simulator = null;
	boolean pathExists = true;
	/*
	JLabel timeLabel = new JLabel();
	long startTime = System.nanoTime();
	public main() {
 		JFrame frame = new JFrame();
 		frame.setSize(850, 500);
 		frame.setLocationRelativeTo(null);
 		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
 		frame.setResizable(false);

 		ButtonGroup group = new ButtonGroup();

 		JRadioButton startSelect = new JRadioButton("Start");
 		JRadioButton obsSelect = new JRadioButton("Add Obstacle");



 		obsSelect.setSelected(true);

 		group.add(startSelect);
 		group.add(obsSelect);


 		JButton find = new JButton("Find Path");
 		find.addActionListener(event -> {
 			findPath();
 			repaint();
 		});

 		JButton reset = new JButton("Reset Path");
 		reset.addActionListener(event -> {
 			resetPath();
 		});

 		SpringLayout layout = new SpringLayout();
 		this.setLayout(layout);
 		layout.putConstraint(SpringLayout.EAST, startSelect, -250, SpringLayout.EAST, this);
 		layout.putConstraint(SpringLayout.NORTH, startSelect, 30, SpringLayout.NORTH, this);
 		 this.add(startSelect);
 		layout.putConstraint(SpringLayout.EAST, find, -60, SpringLayout.EAST, this);
 		layout.putConstraint(SpringLayout.NORTH, find, 120, SpringLayout.NORTH, this);
 		this.add(find);
 		layout.putConstraint(SpringLayout.EAST, reset, 0, SpringLayout.EAST, find);
 		layout.putConstraint(SpringLayout.NORTH, reset, 50, SpringLayout.NORTH, find);
 		this.add(reset);
		
         timeLabel.setAlignmentY(Component.BOTTOM_ALIGNMENT);
 		layout.putConstraint(SpringLayout.EAST, timeLabel,0, SpringLayout.EAST, reset);
 		layout.putConstraint(SpringLayout.SOUTH, timeLabel, 50, SpringLayout.SOUTH, reset);
 		timeLabel.setHorizontalAlignment(JLabel.LEFT);
         timeLabel.setText("Time: 0s");
         timeLabel.setFont(new Font("ARIAL", Font.BOLD, 15));
         timeLabel.setMaximumSize(new Dimension(100,40));
         layout.putConstraint(SpringLayout.SOUTH, timeLabel, 50, SpringLayout.EAST, find);
         this.add(timeLabel);

 		this.addMouseListener(new MouseListener() {
 			@Override
 			public void mousePressed(MouseEvent e) {
 				int x = e.getX();
 				int y = e.getY();
 				System.out.println("X: " + x + ", Y: " + y);
 				Node tile = new Node(new Point(cellWidth * ((int) x / cellWidth), cellHeight * ((int) y / cellHeight)),
 						cellWidth, cellHeight, Direction.UP);

 				switch (e.getButton()) {
 				case MouseEvent.BUTTON1:

 					 set boundaries for selection area
 					if (x >= 500 || (x <= 50 && y >= 450)) {
 						return;
 					}

                 		if(startSelect.isSelected()) {
                             start = tile;
                             tiles.put(tile.getPoint(), Color.GRAY);
                         }
 					else if (obsSelect.isSelected()) {
 						addOrChangeObstacleDirection(x, y);
 					}

 					break;
 				 rc barrier
 				case MouseEvent.BUTTON3:
 					tiles.put(tile.getPoint(), Color.BLACK);
 					closed.add(tile);
 					break;
 				}
 				repaint();
 			}

 			@Override
 			public void mouseClicked(MouseEvent e) {

 			}

 			@Override
 			public void mouseReleased(MouseEvent e) {

 			}

 			@Override
 			public void mouseEntered(MouseEvent e) {

 			}

 			@Override
 			public void mouseExited(MouseEvent e) {

 			}
 		});

 		this.setPreferredSize(frame.getSize());
 		frame.add(this);
 		frame.pack();

 		for (int i = 0; i < width; i += cellWidth) {
 			for (int j = 0; j < height; j += cellHeight) {
 				tiles.put(new Point(i, j), Color.WHITE);
 			}
 		}

 		frame.setVisible(true);
	}

	public void addOrChangeObstacleDirection(int x, int y) {
		//System.out.println("X: " + x + ", Y: " + y);
		Node node = new Node(new Point(cellWidth * ((int) x / cellWidth), cellHeight * ((int) y / cellHeight)),
				cellWidth, cellHeight, Direction.UP);

		if (!inBounds(new Point(x, y)))
			return;
		// if obstacle here, then change dir
		Node exist = obstacles.get(node.getXYPair());
		// Change obstacle orientation
		if (exist != null) {
			exist.rotateSequence();
			//System.out.println("node exists");
			node = exist;
		} else {
			tiles.put(node.getPoint(), Color.GRAY);
		}
		obstacles.put(node.getXYPair(), node);

		repaint();
		
	} */



	public boolean inBounds(Point point) {
		return (point.x >= 0 && point.y >= 0) && (point.x < width && point.y < height);
	}

	public boolean contains(List<Node> list, Node node) {
		for (Node n : list) {
			if (n.equals(node))
				if (n.dir == node.dir || node.dir == Direction.NONE)
				return true;
		}
		return false;
	}

	public boolean contains(Set<Node> list, Node node) {
		for (Node n : list) {
			if (n.equals(node))
				if (n.dir == node.dir || n.dir == Direction.NONE)
				return true;
		}
		return false;
	}

	public void resetPath() {
		//System.out.println("resetted");
		obstacles.clear();
		goals.clear();
		if (simulator != null) simulator.stop();
		//timeLabel.setText("Time: 0s");
		open = new HashSet<Node>();
		closed = new HashSet<Node>();
		robot = new Node(new Point(25, 450), cellWidth, cellHeight);
		tiles = new HashMap<Point, Color>();
		for (int i = 0; i < width; i += cellWidth) {
			for (int j = 0; j < height; j += cellHeight) {
				tiles.put(new Point(i, j), Color.WHITE);
			}
		}
		repaint();

	}
	
	public void updateTime() {
        long stopTime = System.nanoTime();
        //long current = (stopTime - startTime) / (1000 * 1000 * 1000);
        //timeLabel.setText("Time: " + current + " s");
	}
	
	public Node convertJsonToNode(int x, int y, int dir, int id) {
		Point p = new Point(x * 25, 475 - (y * 25));
		
		
		Node node = new Node(p, cellWidth, cellHeight);
		switch (dir) {
			case 0: node.dir = Direction.UP; break;
			case 2: node.dir = Direction.RIGHT; break;
			case 4: node.dir = Direction.DOWN; break;
			case 6: node.dir = Direction.LEFT; break;
			default: node.dir = Direction.UP; break;
		}
		return node;
	}
	public String changeDirection(Direction parentdir, Direction currdir, ArrayList<String> list) {
		if (parentdir == Direction.UP ) {
			if (currdir == Direction.LEFT) return "FL90";
			if (currdir == Direction.RIGHT) return "FR90";
			if (currdir == Direction.DOWN) return "BW01";
		}
		if (parentdir == Direction.DOWN) {
			if (list.get(list.size()-1) == "BW01") {
				if (currdir == Direction.LEFT) return "BL90";
				if (currdir == Direction.RIGHT) return "BR90";
			}
			if (currdir == Direction.LEFT) return "FL90";
			if (currdir == Direction.RIGHT) return "FR90";
		}
		if (parentdir == Direction.LEFT ) {
			if (currdir == Direction.UP) return "FL90";
			if (currdir == Direction.DOWN) return "FR90";
			if (currdir == Direction.RIGHT) return "BW01";
		}
		if (parentdir == Direction.RIGHT) {
			if (currdir == Direction.UP) return "FL90";
			if (currdir == Direction.DOWN) return "FR90";
			if (currdir == Direction.LEFT) return "BW01";
		}
		if (parentdir == Direction.NONE) {
			return "FW01";
		}
		return "WHYCAMEHERE";
	}
	
	public String turnToNextSide(String json) {
		  JSONObject returnObj = new JSONObject();
		        JSONArray list = new JSONArray();
		        list.add("FW01");
		        list.add("FW01");
		        list.add("FR90");
		        list.add("FW01");
		        list.add("FW01");
		        list.add("SNAP");
		        returnObj.put("commands", list);
		  return returnObj.toJSONString();
		 }
	
	public Node convertJsonToNode(JSONObject jsonNode) {
		Long x = (Long) jsonNode.get("x");
		Long y = (Long) jsonNode.get("y");
		Long dir = (Long) jsonNode.get("d");
		Point p = new Point(x.intValue() * 25, 475 - (y.intValue() * 25));
		
		
		Node node = new Node(p, cellWidth, cellHeight);
		switch (dir.intValue()) {
			case 0: node.dir = Direction.UP; break;
			case 2: node.dir = Direction.RIGHT; break;
			case 4: node.dir = Direction.DOWN; break;
			case 6: node.dir = Direction.LEFT; break;
			default: node.dir = Direction.UP; break;
		}
		return node;
	}
	
	public JSONObject getJsonPath(List<Node> pathNodes) {
		JSONObject returnObj = new JSONObject();


        JSONArray list = new JSONArray();
        int counter = 1;
        int currX = -100;
        int currY = -100;
        int prevX = -100;
        int prevY = -100;
        for (int i =0; i< pathNodes.size(); i++) {
        	

			
        	currX = pathNodes.get(i).getPoint().x;
			currY = pathNodes.get(i).getPoint().y;
			
			if (i != pathNodes.size()-1 && pathNodes.get(i).getDirection() != pathNodes.get(i+1).getDirection() ) {
				//System.out.println("NODE PARENT DIR: " + node.getParent().getDirection() + ", NODE DIR: " + node.getDirection());
				String turn = "";
				turn = changeDirection(pathNodes.get(i).getDirection(),pathNodes.get(i+1).getDirection(), list);
				list.add(turn);
			}
			else if (i == pathNodes.size()-1 && pathNodes.get(i).getDirection() != pathNodes.get(i-1).getDirection()) {
				String turn = "";
				turn = changeDirection(pathNodes.get(i).getDirection(),pathNodes.get(i-1).getDirection(), list);
				list.add(turn);
			}
			//basic movement
			else {
				switch (pathNodes.get(i).getDirection()) {
				
				case UP: list.add("FW"); break;
				case RIGHT: list.add("FW"); break;
				case LEFT: 
					if (currX > pathNodes.get(i-1).getPoint().x) list.add("BW");
					else list.add("FW"); 
					break;
				case DOWN: 
					if (currY < pathNodes.get(i-1).getPoint().y) list.add("BW");
					else if (list.get(list.size()- 1) == "BW") list.add("BW");
					else list.add("FW");
					break;
				}	
			}
			
			//check if reached a obstacle then do action
			for (Node n : obstacles.values()) {
				//System.out.println("node XY: " + node.getXYPair() + ", n XY: " + n.getActualGoal().getXYPair());
				if ( pathNodes.get(i).getXYPair().equals(n.getActualGoal().getXYPair()) &&
						pathNodes.get(i).getDirection() == n.getActualGoal().getDirection()) {
					list.add("SNAP" + counter);
					counter++;
					//n.setVisited(true);
					break;
				}
			}
        }

		list.add("FIN");
		System.out.println("ORIGINAL" + list);
		for (int i =0; i < list.size(); i++) {
			if (list.get(i) == "FW" || list.get(i) == "BW") {
				if (list.get(i+1) == "FR90" || list.get(i+1) == "FL90") {
					String val = list.get(i) == "FW" ? "FW01" : "BW01";
					// if after turn is not FW then remove after that
					if (list.get(i+2).toString().contains("SNAP") ) {
						if (list.get(i+3) == "FW") {
							list.remove(i+3);
						}
					}
					else {
						list.remove(i+2);	
					}
					//System.out.println("Removed: " + list.get(i));
					
					list.remove(i);	
				}
			}
		}
		System.out.println("EDIT 1" + list);
		int noTracker = 0;
		int bwOrFw = 0;
		JSONArray list2 = new JSONArray();
		for (int i = 0; i < list.size(); i++) {
			if (list.get(i) == "FW" || list.get(i) == "BW") {
				bwOrFw = list.get(i) == "FW" ? 1: 2;
				noTracker++;
				
			}
			else {
				if (bwOrFw == 1) list2.add(noTracker < 10 ? "FW0" + noTracker : "FW" + noTracker);
				else if (bwOrFw == 2) list2.add(noTracker < 10 ? "BW0" + noTracker : "BW" + noTracker);
				list2.add(list.get(i));
				noTracker = 0;
				bwOrFw = 0;
			}
			//endindex++;
		}
		returnObj.put("commands", list2);
		System.out.println(returnObj);
		return returnObj;
	}
	@SuppressWarnings("unchecked")
	public String findPathJson(JSONObject jsonObject) {
		ArrayList<LinkedHashMap<String, String>> arrs = (ArrayList) jsonObject.get("obstacles");
		for (LinkedHashMap obj : arrs) {
        	Node newNode = convertJsonToNode((int) obj.get("x"), (int) obj.get("y"), (int) obj.get("d"), (int) obj.get("id"));
        	obstacles.put(newNode.getXYPair(), newNode);
		}
        Node goal = findPath();
        Node current = goal;
		List<Node> pathNodes = new ArrayList();
		while (!current.equals(start)) {
				pathNodes.add(0, current);
				current = current.getParent();
		}
		JSONObject returnObj = getJsonPath(pathNodes);
		return returnObj.toJSONString();
	}
	public Node findPath() {
		//startTime = System.nanoTime();
		// have to copy to a new var, cant just use obstacle.values as it is being used
		// to draw the nodes
		Collection<Node> nodes = new HashSet<Node>(obstacles.values().size());
		Iterator<Node> iterator = obstacles.values().iterator();
		while (iterator.hasNext()) {
			nodes.add(iterator.next().clone());
		}
		Node startNode = start;
		while (!nodes.isEmpty()) {
			Node end = startNode.getNearest(nodes);
			nodes.remove(end);
			try {
				closed = new HashSet<Node>();
				startNode = findPathStartToEndNode(startNode, end);
			}
			catch (InterruptedException ex) {
				ex.printStackTrace();
			} 
			open = new HashSet<Node>();
			//closed = new HashSet<Node>();
		}
	final Node copiedNode = startNode;
//	drawPathAlgo(copiedNode);
//		simulator = new Thread(() -> {
//
//		try {
//			drawPath(copiedNode);
//		} catch (Exception e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//	});
//		simulator.start();
		return copiedNode;
		
	}

	public Node findPathStartToEndNode(Node start, Node end) throws InterruptedException {
		open.add(start);
		closed.add(end);
		//closed.add(new Node(new Point(100, 425), cellWidth, cellHeight));
		
//		closed.add(new Node(new Point(100, 400), cellWidth, cellHeight));
//		closed.add(new Node(new Point(100, 425), cellWidth, cellHeight));
//		closed.add(new Node(new Point(100, 450), cellWidth, cellHeight));
//		closed.add(new Node(new Point(100, 475), cellWidth, cellHeight));
//		closed.add(new Node(new Point(75, 400), cellWidth, cellHeight));
//		closed.add(new Node(new Point(75, 425), cellWidth, cellHeight));
//		closed.add(new Node(new Point(75, 450), cellWidth, cellHeight));
//		closed.add(new Node(new Point(75, 475), cellWidth, cellHeight));
//		closed.add(new Node(new Point(50, 400), cellWidth, cellHeight));
//		closed.add(new Node(new Point(50, 425), cellWidth, cellHeight));
//		closed.add(new Node(new Point(50, 450), cellWidth, cellHeight));
//		closed.add(new Node(new Point(50, 475), cellWidth, cellHeight));
		while (!open.isEmpty()) {
			Map<Node, Double> costs = new HashMap<Node, Double>();
			for (Node node : open) {
				costs.put(node, node.cost());
			}
			Node current = Collections.min(costs.entrySet(), Map.Entry.comparingByValue()).getKey();
			Node actualGoal = end.getActualGoal();
			goals.put(actualGoal.getXYPair(), actualGoal);
			open.remove(current);
			
			// closed set contains values wont consider again, so barriers or solved ones
			// closed set need consider direction, same coordinates with diff DIR
			// NOT SAME
			Node currentNodeWithDir = current.clone();
			closed.add(currentNodeWithDir);
			
			for (Node n : end.getSurroundingNodes()) {
				closed.add(n);
			}
			// if current == end then found goal, end loop
			
			if (current.equals(actualGoal) && current.dir == actualGoal.dir) {
				// obstacles.put(actualGoal.getXYPair(), current);
//				System.out.println("came here");
//				drawPath(current);
				return current;
			}

			//
			for (Node adjacent : current.getAdjacentNodes(open)) {
				if (contains(closed, adjacent) || !inBounds(adjacent.getPoint()))
					continue;
				//System.out.println("ADJACENT DIR " + adjacent.dir);
				double newG = current.g + adjacent.getHeuristic(current);
				if (newG < adjacent.g || !contains(open, adjacent)) {
					adjacent.g = newG;
					adjacent.h = adjacent.getHeuristic(actualGoal);
					adjacent.parent = current;

					if (!contains(open, adjacent))
						open.add(adjacent);
				}
			}
		}
		return end;
	}

	
	public void updateMovement(Node node) throws InterruptedException {
		robot = node;
		// for each obstacle
		// if robot is in its actual goal, set is visited true
		for (Node n : obstacles.values()) {
			//System.out.println("node XY: " + node.getXYPair() + ", n XY: " + n.getActualGoal().getXYPair());
			if (node.getXYPair().equals(n.getActualGoal().getXYPair()) &&
					node.getDirection() == n.getActualGoal().getDirection()) {

				n.setVisited(true);
			}
		}
		//System.out.println("DIRECTION OF ROBOT: " + robot.dir);
		updateTime();
		repaint();
		Thread.sleep(500);
		
	}	
	
	public void drawPathAlgo(Node actualGoal) {
		if (pathExists) {
			Node current = actualGoal;
			
			while (!current.equals(start)) {
				//System.out.println("Current: X" + current.getParent().getPoint().x + ", Y:" + current.getParent().getPoint().y);
				System.out.println("Draw Path X: " + current.getParent().getPoint().getX()+" Path Y: " + current.getParent().getPoint().getY());
				tiles.put(current.getParent().getPoint(), Color.GRAY);

				
				//Node node = new Node(new Point(current.getParent().getPoint().x, current.getParent().getPoint().y), cellWidth, cellHeight);
				//robot.setPoint();
				//validate(); 

				
//				System.out.println("Robot: X" + robot.getPoint().x + ", Y:" + robot.getPoint().y);
				
				current = current.getParent();
			}
		}
		tiles.put(actualGoal.getPoint(), Color.GREEN);
		repaint();
	}

	public void drawPath(Node actualGoal) {
		if (pathExists) {
			Node current = actualGoal;
			List<Node> pathNodes = new ArrayList();
			// move nodes to a list
			while (!current.equals(start)) {
				pathNodes.add(0, current);
				current = current.getParent();
			}
			for (Node n : pathNodes) {
				try {
					updateMovement(n);
				} catch (Exception ex) {
					ex.printStackTrace();
				}
			}
			
//			while (!current.equals(start)) {
//				System.out.println("Current: X" + current.getParent().getPoint().x + ", Y:" + current.getParent().getPoint().y);
//				//tiles.put(current.getParent().getPoint(), Color.GRAY);
//				Node node = new Node(new Point(current.getParent().getPoint().x, current.getParent().getPoint().y), cellWidth, cellHeight);
//				//robot.setPoint();
//				//validate(); 
//				 
//				try {
//
//					updateMovement(node);
//				} catch (Exception ex) {
//					ex.printStackTrace();
//				}
//				
//				System.out.println("Robot: X" + robot.getPoint().x + ", Y:" + robot.getPoint().y);
//				
//				current = current.getParent();
//			}
		}
		repaint();
	}

	private void drawObstacle(Graphics g) {
		// Set obstacles
		for (Node obs : obstacles.values()) {
			//System.out.println("OBS:" + obs.getXYPair());
			g.setColor(Color.BLACK);
			g.fillRect(obs.getPoint().x, obs.getPoint().y, cellWidth, cellHeight);
			int heightOfImage = cellWidth / 4;

			if (obs.visited)
				g.setColor(Color.decode("#a5be002"));
			else
				g.setColor(Color.decode("#ff0700"));
			//System.out.println("obs dir: " + obs.dir);
			if (obs.dir == Direction.UP)
				g.fillRect(obs.getPoint().x, obs.getPoint().y, cellWidth, heightOfImage);
			else if (obs.dir == Direction.DOWN)
				g.fillRect(obs.getPoint().x, obs.getPoint().y + cellWidth - 5, cellWidth, heightOfImage);
			else if (obs.dir == Direction.LEFT)
				g.fillRect(obs.getPoint().x, obs.getPoint().y, 5, cellWidth);
			else if (obs.dir == Direction.RIGHT)
				g.fillRect(obs.getPoint().x + cellWidth - 5, obs.getPoint().y, heightOfImage, cellWidth);

		}
	}
	@Override
	public void paintComponent(Graphics tool) {
		super.paintComponent(tool);
		//System.out.println("REPAINTING");

		// Draw path of the robot
		for (Map.Entry<Point, Color> tile : tiles.entrySet()) {
			tool.setColor(tile.getValue());
			tool.fillRect(tile.getKey().x, tile.getKey().y, cellWidth, cellHeight);
		}
		// start area
		drawStartingGrid(tool);
		tool.setColor(Color.BLACK);
		for (int i = 0; i < width; i += cellWidth) {
			tool.drawLine(i, 0, i, height);
		}
		for (int i = 0; i < height; i += cellHeight) {
			tool.drawLine(0, i, width, i);
		}

		if (start != null) {
			tool.setColor(Color.GREEN);
//            tool.fillRect(start.point.x, start.point.y, cellWidth, cellHeight);
			tool.fillOval(start.getPoint().x + 8, start.getPoint().y + 8, 10, 10);
		}
		


		drawRobot(tool);
		drawObstacle(tool);
	}

	private void drawRobot(Graphics tool) {
		tool.setColor(Color.GRAY);
		int x = robot.getPoint().x -12;
		int y = robot.getPoint().y-26;

		Direction robotDir = robot.getDirection();

		//Direction robotDir = Direction.DOWN;
		//System.out.println("UPdated robot X: " + x + ", Y: " + y);

		switch(robotDir){
			case UP:
				tool.fillRect(x, y, 50, 75);

				// Wheels Front Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+10, 7, 20);
		
				// Wheels Front Right
				tool.setColor(Color.black);
				tool.fillRect(x+50, y+10, 7, 20);
		
				// Wheels Back Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+51, 7, 20);
		
				// Wheels Back Right
				tool.setColor(Color.black);
				tool.fillRect(x+50, y+51, 7, 20);
				
				// Front Sensor
				tool.setColor(Color.yellow);
				tool.fillRect(x+20, y, 10, 10);
		
				// Camera
				tool.setColor(Color.green);
				tool.fillRect(x+40, y+35, 10, 10);

				break;
			case DOWN:
				tool.fillRect(x, y, 50, 75);

				// Wheels Front Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+10, 7, 20);
		
				// Wheels Front Right
				tool.setColor(Color.black);
				tool.fillRect(x+50, y+10, 7, 20);
		
				// Wheels Back Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+51, 7, 20);
		
				// Wheels Back Right
				tool.setColor(Color.black);
				tool.fillRect(x+50, y+51, 7, 20);
				
				// Front Sensor
				tool.setColor(Color.yellow);
				tool.fillRect(x+20, y+64, 10, 10);
		
				// Camera
				tool.setColor(Color.green);
				tool.fillRect(x, y+35, 10, 10);

				break;

			case LEFT:
				tool.fillRect(x-12, y+15, 75, 50);

				// Wheels Front Right
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+8, 20, 7);
		
				// Wheels Back Right
				tool.setColor(Color.black);
				tool.fillRect(x+35, y+8, 20, 7);
		
				// Wheels Front Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+65, 20, 7);
		
				// Wheels Back Left
				tool.setColor(Color.black);
				tool.fillRect(x+35, y+65, 20, 7);
				
				// Front Sensor
				tool.setColor(Color.yellow);
				tool.fillRect(x-11, y+35, 10, 10);
		
				// Camera
				tool.setColor(Color.green);
				tool.fillRect(x+20, y+15, 10, 10);

				break;

			case RIGHT:
				tool.fillRect(x-12, y+15, 75, 50);

				// Wheels Front Right
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+8, 20, 7);
		
				// Wheels Back Right
				tool.setColor(Color.black);
				tool.fillRect(x+35, y+8, 20, 7);
		
				// Wheels Front Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+65, 20, 7);
		
				// Wheels Back Left
				tool.setColor(Color.black);
				tool.fillRect(x+35, y+65, 20, 7);
				
				// Front Sensor
				tool.setColor(Color.yellow);
				tool.fillRect(x+53, y+35, 10, 10);
		
				// Camera
				tool.setColor(Color.green);
				tool.fillRect(x+20, y+55, 10, 10);

				break;
			// Default will be up
			default:
				tool.fillRect(x, y, 50, 75);

				// Wheels Front Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+10, 7, 20);
		
				// Wheels Front Right
				tool.setColor(Color.black);
				tool.fillRect(x+50, y+10, 7, 20);
		
				// Wheels Back Left
				tool.setColor(Color.black);
				tool.fillRect(x-7, y+51, 7, 20);
		
				// Wheels Back Right
				tool.setColor(Color.black);
				tool.fillRect(x+50, y+51, 7, 20);
				
				// Front Sensor
				tool.setColor(Color.yellow);
				tool.fillRect(x+20, y, 10, 10);
		
				// Camera
				tool.setColor(Color.green);
				tool.fillRect(x+40, y+35, 10, 10);

				break;
		}
        
	}

	private void drawStartingGrid(Graphics tool) {
		// start area
		tool.setColor(Color.LIGHT_GRAY);
		tool.fillRect(0, 400, cellWidth, cellHeight);
		tool.fillRect(0, 425, cellWidth, cellHeight);
		tool.fillRect(0, 450, cellWidth, cellHeight);
		tool.fillRect(0, 475, cellWidth, cellHeight);

		tool.fillRect(25, 400, cellWidth, cellHeight);
		tool.fillRect(25, 425, cellWidth, cellHeight);
		tool.fillRect(25, 450, cellWidth, cellHeight);
		tool.fillRect(25, 475, cellWidth, cellHeight);

		tool.fillRect(50, 400, cellWidth, cellHeight);
		tool.fillRect(50, 425, cellWidth, cellHeight);
		tool.fillRect(50, 450, cellWidth, cellHeight);
		tool.fillRect(50, 475, cellWidth, cellHeight);

		tool.fillRect(75, 400, cellWidth, cellHeight);
		tool.fillRect(75, 425, cellWidth, cellHeight);
		tool.fillRect(75, 450, cellWidth, cellHeight);
		tool.fillRect(75, 475, cellWidth, cellHeight);
	}
	public String callAlgo(JSONObject input) {
		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":[{\"x\":16,\"y\":18,\"id\":2,\"d\":2},{\"x\":14,\"y\":11,\"id\":3,\"d\":4},{\"x\":7,\"y\":6,\"id\":4,\"d\":2},{\"x\":17,\"y\":6,\"id\":5,\"d\":0},{\"x\":8,\"y\":15,\"id\":6,\"d\":6}],\"mode\":\"0\"}}";
		
		// System.out.println(input);
		// Object obj=JSONValue.parse(input);  
		// JSONObject jsonObject = (JSONObject) obj;  
		 //System.out.println(jsonObject.get("cat"));
		 //System.out.println(jsonObject.get("value"));
		 //JSONObject value = (JSONObject) jsonObject.get("value");
		 //System.out.println(value.get("mode"));
		String outputjson = findPathJson(input);
		return outputjson;
		}
//	 public static void main(String[] args) {
//	 	main simulator = new main();
//	 	//String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":[{\"x\":3,\"y\":15,\"id\":1,\"d\":4},{\"x\":16,\"y\":18,\"id\":2,\"d\":2},{\"x\":14,\"y\":11,\"id\":3,\"d\":4},{\"x\":7,\"y\":6,\"id\":4,\"d\":2},{\"x\":17,\"y\":6,\"id\":5,\"d\":0},{\"x\":8,\"y\":15,\"id\":6,\"d\":6}],\"mode\":\"0\"}}";
//	 	String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":[{\"x\":16,\"y\":18,\"id\":2,\"d\":2},{\"x\":14,\"y\":11,\"id\":3,\"d\":4},{\"x\":7,\"y\":6,\"id\":4,\"d\":2},{\"x\":17,\"y\":6,\"id\":5,\"d\":0},{\"x\":8,\"y\":15,\"id\":6,\"d\":6}],\"mode\":\"0\"}}";
//		
//	 	String outputjson = simulator.findPathJson(json);
//	 }

}