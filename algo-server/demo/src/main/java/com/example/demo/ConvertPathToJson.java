package com.example.demo;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class ConvertPathToJson {
	

	//converting path found to robots command logic
	public static JSONObject getJsonPath(List<Node> pathNodes, Collection<Node> obstacleNodes) {
		JSONObject returnObj = new JSONObject();


        JSONArray list = new JSONArray();
        int counter = 1;
        for (int i =0; i< pathNodes.size(); i++) {
        	//System.out.println(pathNodes.get(i).getXYPair() + ",RobotFacing:" + pathNodes.get(i).robotdir + ",RobotNode:" + pathNodes.get(i).getDirection());
			// if not last node and direction diff
			// add snap in first, add a FW if turn after goal, reverse and turn is added in EDIT part
			// add in turn direction
			if (i != pathNodes.size()-1 && pathNodes.get(i).robotdir != pathNodes.get(i+1).robotdir ) {
				//System.out.println("hit here");
				for (Node n : obstacleNodes) {
					if ( pathNodes.get(i).getXYPair().equals(n.getActualGoal().getXYPair()) &&
							pathNodes.get(i).robotdir == n.getActualGoal().robotdir) {
						//if future change dir then add a forward or backward here first to snap depending on robotfacing and node,
						list.add(pathNodes.get(i).robotdir == pathNodes.get(i).dir ? "FW" : "BW");
						list.add("SNAP" + n.id);
						counter++;
						//n.setVisited(true);
						break;
					}
				}
				String turn = "";
				turn = changeDirection(pathNodes.get(i),pathNodes.get(i+1), list);
				list.add(turn);
				
				
			}
			// if is last node and direction diff
			else if (i == pathNodes.size()-1 && pathNodes.get(i).robotdir != pathNodes.get(i-1).robotdir) {
				//String turn = "";
				//turn = changeDirection(pathNodes.get(i).getDirection(),pathNodes.get(i-1).getDirection(), list);
				//list.add(turn);
				//check if reached a obstacle then do action
				for (Node n : obstacleNodes) {
					//System.out.println("node XY: " + node.getXYPair() + ", n XY: " + n.getActualGoal().getXYPair());
					if ( pathNodes.get(i).getXYPair().equals(n.getActualGoal().getXYPair()) &&
							pathNodes.get(i).robotdir == n.getActualGoal().robotdir) {
						list.add("SNAP" + n.id);
						counter++;
						//n.setVisited(true);
						break;
					}
				}
			}
			//basic movement
			else {
				switch (pathNodes.get(i).robotdir) {
//				case RIGHT: list.add("FW"); break;
//				case LEFT: 
//					if (pathNodes.get(i).dir != pathNodes.get(i).robotdir) list.add("BW");
//					else list.add("FW"); 
//					break;
				case DOWN: 
					if (pathNodes.get(i).getPoint().y < pathNodes.get(i-1).getPoint().y) list.add("BW");
					else if (list.get(list.size()- 1) == "BW") list.add("BW");
					else list.add("FW");
					break;
				default: 
					if (pathNodes.get(i).dir != pathNodes.get(i).robotdir) list.add("BW");
					else list.add("FW");  
					break;
				}
				//check if reached a obstacle then do action
				for (Node n : obstacleNodes) {
					//System.out.println("node XY: " + node.getXYPair() + ", n XY: " + n.getActualGoal().getXYPair());
					if ( pathNodes.get(i).getXYPair().equals(n.getActualGoal().getXYPair()) &&
							pathNodes.get(i).robotdir == n.getActualGoal().robotdir) {
						list.add("SNAP" + n.id);
						counter++;
						//n.setVisited(true);
						break;
					}
				}
			}
			

        }

		list.add("FIN");
		System.out.println("ORIGINAL" + list);
		for (int i =0; i < list.size(); i++) {
			if (list.get(i).toString().contains("SNAP")) {
				if (list.get(i+1) == "FR90" || list.get(i+1) == "FL90") {
					//list.add(i+1, "BW");
				}
				//add if turn is BR90/FL90 == negative do forward
			}
			if (list.get(i) == "FR90" && list.get(i+1) == "FL90") {
				list.remove(i);
				list.remove(i);
				list.add(i, "FRL");
			}
			else if (list.get(i) == "FL90" && list.get(i+1) == "FR90") {
				list.remove(i);
				list.remove(i);
				list.add(i, "FLR");
			}
			if (list.get(i) == "BR90" && list.get(i+1) == "BL90") {
				list.remove(i);
				list.remove(i);
				list.add(i, "BRL");
			}
			else if (list.get(i) == "BL90" && list.get(i+1) == "BR90") {
				list.remove(i);
				list.remove(i);
				list.add(i, "BLR");
			}
			if (list.get(i) == "FR90" || list.get(i) == "FL90" || list.get(i) == "FRL" || list.get(i) == "FLR") {
				if ((list.get(i+1) == "FW" || list.get(i+1) == "BW")) {
					list.remove(i+1);
			}
			}
		}
		System.out.println("EDIT 1" + list);
		int noTracker = 1;
		String prevcommand = "";
		String command = "";
		JSONArray list2 = new JSONArray();
		for (int i = 0; i < list.size(); i++) {
			command = list.get(i).toString();
			
			//if last index just add, since is always FIN
			if (i == list.size() - 1) {
				list2.add(list.get(i));
				break;

			}
			
			if (command == list.get(i+1).toString()) {
				noTracker++;
			}
			else {
				if (command == "FW") {
					list2.add(noTracker < 10 ? "FW0" + noTracker : "FW" + noTracker);
				}
				else if (command == "BW") {
					list2.add(noTracker < 10 ? "BW0" + noTracker : "BW" + noTracker);
				}
				else {
					list2.add(list.get(i));
				}
				noTracker = 1;
					
			}
		}
		returnObj.put("commands", list2);
		System.out.println(returnObj);
		return returnObj;
	}
	
	
	
	public static String changeDirection(Node currNode, Node nextNode, ArrayList<String> list) {
		Direction currdir = currNode.robotdir;
		Direction nextdir = nextNode.robotdir;
		if (currdir == Direction.UP) {
			if (currNode.dir == Direction.DOWN) {
				if (nextNode.dir == Direction.LEFT) {
					list.add("BR90");
					return "FW";
					
				}
				if (nextNode.dir == Direction.RIGHT) {
					list.add("BL90");
					return "FW";		
				}
			}
			if (nextdir == Direction.LEFT) return "FL90";
			if (nextdir == Direction.RIGHT) return "FR90";
			if (nextdir == Direction.DOWN) return "BW";
		}
		if (currdir == Direction.DOWN) {
			if (currNode.dir == Direction.UP) {
				if (nextNode.dir == Direction.LEFT) {
					list.add("BL90");
					return "FW";
					
				}
				if (nextNode.dir == Direction.RIGHT) {
					list.add("BR90");
					return "FW";		
				}
			}
			if (list.get(list.size()-1) == "BW") {
				if (nextdir == Direction.LEFT) return "BL90";
				if (nextdir == Direction.RIGHT) return "BR90";	
			}
			if (nextdir == Direction.LEFT) return "FR90";
			if (nextdir == Direction.RIGHT) return "FL90";	
		}
		if (currdir == Direction.RIGHT) {
			if (currNode.dir == Direction.LEFT) {
				if (nextNode.dir == Direction.UP) {
					list.add("BR90");
					return "FW";
					
				}
				if (nextNode.dir == Direction.DOWN) {
					list.add("BL90");
					return "FW";		
				}
			}
			if (nextdir == Direction.UP) return "FL90";
			if (nextdir == Direction.DOWN) return "FR90";	
			if (nextdir == Direction.LEFT) return "BW01";
		}
		if (currdir == Direction.LEFT) {
			// going backwards
			if (currNode.dir == Direction.RIGHT) {
				if (nextNode.dir == Direction.UP) {
					list.add("BL90");
					return "FW";
					
				}
				if (nextNode.dir == Direction.DOWN) {
					list.add("BR90");
					return "FW";		
				}
			}
			if (nextdir == Direction.UP) return "FR90";
			if (nextdir == Direction.DOWN) return "FL90";
			if (nextdir == Direction.RIGHT) return "BW01";
		}

		if (currdir ==  Direction.NONE) return "FW";
		return "WHYCAMEHERE";
	}
}
