package com.example.demo;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class Node implements Cloneable {
        private Point point;
        private int cellWidth;
        private int cellHeight;
        public Direction dir;
        public double g, h;
        public boolean visited;
        public Node parent;
        public Node child;

        public Node(Point point, int cw, int ch) {
            this.point = point;
            this.cellWidth = cw;
            this.cellHeight = ch;
            this.visited = false;
            this.dir = Direction.NONE;
        }
        
        public Node(Point point, int cw, int ch, Direction dir) {
            this.point = point;
            this.cellWidth = cw;
            this.cellHeight = ch;
            this.visited = false;
            this.dir = dir;
        }
        
		public Node getParent() {
            return parent;
        }
		
		public void setVisited(boolean visit) {
			visited = visit;
		}

        public Direction getDirection() {
            return dir;
        }
        
        public double cost() {
            return g + h;
        }
        
        public void setDir(Direction dir) {
            this.dir = dir;
        }
        
        // add in previous node and if previous node direction change, means more weight
        public double getHeuristic(Node node) {
            int x = Math.abs((point.x - node.point.x)/cellWidth);
            int y = Math.abs((point.y - node.point.y)/cellHeight);
            int additionalWeight = 0;
            Direction currentDir = getDirectionOfTwoNodes(this.parent, this);
            Direction futureDir = getDirectionOfTwoNodes(this, node);
            
            if (currentDir != futureDir) additionalWeight += 100;
            //System.out.println("Additional Weight" + additionalWeight);
            if(x > y)
                return 28*y + 10*(x - y);
            return 28*x + 10*(y - x);       
        }
        
        public List<Node> getAdjacentNodes(Set<Node> openNodes) {
            List<Node> nodes = new LinkedList<Node>();
            // BR
            //nodes.add(new Node(new Point(point.x + cellWidth, point.y + cellHeight), this.cellWidth, this.cellHeight));
            // R
            if (!(dir == Direction.LEFT)) {
            	 nodes.add(new Node(new Point(point.x + cellWidth, point.y), this.cellWidth, this.cellHeight, Direction.RIGHT));    
            }
            else {
            	nodes.add(new Node(new Point(point.x + cellWidth, point.y), this.cellWidth, this.cellHeight, Direction.LEFT));
            }
           // TR
            //nodes.add(new Node(new Point(point.x + cellWidth, point.y - cellHeight), this.cellWidth, this.cellHeight));
            // T
            if (!(dir == Direction.DOWN)) {
            	nodes.add(new Node(new Point(point.x, point.y - cellHeight), this.cellWidth, this.cellHeight, Direction.UP));   
            }
            else {
            	nodes.add(new Node(new Point(point.x, point.y - cellHeight), this.cellWidth, this.cellHeight, Direction.DOWN));
            }
            // TL
            //nodes.add(new Node(new Point(point.x - cellWidth, point.y - cellHeight), this.cellWidth, this.cellHeight));
            // L
            if (!(dir == Direction.RIGHT)) {
            	nodes.add(new Node(new Point(point.x - cellWidth, point.y), this.cellWidth, this.cellHeight, Direction.LEFT));  	
            }
            else {
            	nodes.add(new Node(new Point(point.x - cellWidth, point.y), this.cellWidth, this.cellHeight, Direction.RIGHT));
            }
             // BL
            //nodes.add(new Node(new Point(point.x - cellWidth, point.y + cellHeight), this.cellWidth, this.cellHeight));
            // B
            if (!(dir == Direction.UP)) {
            	 nodes.add(new Node(new Point(point.x, point.y + cellHeight), this.cellWidth, this.cellHeight, Direction.DOWN));   
            }
            else {
            	nodes.add(new Node(new Point(point.x, point.y + cellHeight), this.cellWidth, this.cellHeight, Direction.UP));
            }
            // hmm this part can probably optimise? should have a better way to find neighbour nodes in adjacent nodes
            // without creating new nodes to compare
            List<Node> correctNodes = new ArrayList<Node>();
            //List<Node> remove = new ArrayList<Node>();
            
//            for(Node node: nodes) {
//                for(Node openNode: openNodes) {
//                    if(node.equals(openNode)) {
//                        correctNodes.add(openNode);
//                        //remove.add(node);
//                        break;
//                    }
//                }
//            }
            
            //nodes.removeAll(remove);
            for(Node node: nodes) {
                node.g = this.g + getHeuristic(node);
                correctNodes.add(node);
            }
            return correctNodes;
        }
        
        public Direction getDirectionOfTwoNodes(Node prev, Node current) {
        	if (prev == null) return null;
        	
        	if (prev.getPoint().x == current.getPoint().x-this.cellWidth && prev.getPoint().y == current.getPoint().y) {
        		return Direction.RIGHT;
        	}
        	if (prev.getPoint().x == current.getPoint().x+this.cellWidth && prev.getPoint().y == current.getPoint().y) {
        		return Direction.LEFT;
        	}
        	if (prev.getPoint().x == current.getPoint().x && prev.getPoint().y == current.getPoint().y - this.cellWidth) {
        		return Direction.DOWN;
        	}
        	if (prev.getPoint().x == current.getPoint().x && prev.getPoint().y == current.getPoint().y + this.cellWidth) {
        		return Direction.UP;
        	}
        	return null;
        }
        
        public Point getPoint() {
            return point;
        }

        public void setPoint(Point point2) {
            this.point = point2;
        }
        
        public boolean equals(Node node) {
            return point.x == node.point.x && point.y == node.point.y;
        }
        
        public String toString() {
            return point.toString();
        }
        
        public String getXYPair() {
            return point.x + ", " + point.y;
        }
        
    	public Node getNearest(Collection<Node> nodes) {
    		double nearest = Double.MAX_VALUE;
    		Node x = null;
    		for (Node n : nodes) {
    			double cost = this.getHeuristic(n);
    			if (cost < nearest) {
    				nearest = cost;
    				x = n;
    			}
    		}
    		return x;
    	}
    	
    	public Node getActualGoal() {
    		Node actualGoal = new Node(new Point(this.getPoint().x, this.getPoint().y), cellWidth, cellHeight);
    		//System.out.println("Current Points = " + actualGoal.getPoint().x + ", " + actualGoal.getPoint().y);
    		switch (this.dir) {
    		case LEFT:
    			actualGoal.getPoint().x -= (2 * cellWidth);
    			actualGoal.dir = Direction.UP;
    			break;
    		case RIGHT:
    			actualGoal.getPoint().x += (2 * cellWidth);
    			actualGoal.dir = Direction.DOWN;
    			break;
    		case UP:
    			actualGoal.getPoint().y -= (2 * cellWidth);
    			actualGoal.dir = Direction.RIGHT;
    			break;
    		case DOWN:
    			actualGoal.getPoint().y += (2 * cellWidth);
    			actualGoal.dir = Direction.LEFT;
    			break;
    		}
    		//System.out.println("After Points = " + actualGoal.getPoint().x + ", " + actualGoal.getPoint().y);
    		return actualGoal;
    	}
        
        public List<Node> getSurroundingNodes() {

            List<Node> nodes = new LinkedList<Node>();
            // TR
            nodes.add(new Node(new Point(point.x + cellWidth, point.y + cellHeight), this.cellWidth, this.cellHeight));
            // R
            nodes.add(new Node(new Point(point.x + cellWidth, point.y), this.cellWidth, this.cellHeight));
            // BR
            nodes.add(new Node(new Point(point.x + cellWidth, point.y - cellHeight), this.cellWidth, this.cellHeight));
            // B
            nodes.add(new Node(new Point(point.x, point.y - cellHeight), this.cellWidth, this.cellHeight));
            // BL
            nodes.add(new Node(new Point(point.x - cellWidth, point.y - cellHeight), this.cellWidth, this.cellHeight));
            // L
            nodes.add(new Node(new Point(point.x - cellWidth, point.y), this.cellWidth, this.cellHeight));
            // TL
            nodes.add(new Node(new Point(point.x - cellWidth, point.y + cellHeight), this.cellWidth, this.cellHeight));
            // T
            nodes.add(new Node(new Point(point.x, point.y + cellHeight), this.cellWidth, this.cellHeight));
            
            return nodes;
        }
        
    	public void rotateSequence() {
    		if (this.dir == Direction.UP)
    			this.dir = Direction.RIGHT;
    		else if (dir == Direction.RIGHT)
    			this.dir = Direction.DOWN;
    		else if (dir == Direction.DOWN)
    			this.dir = Direction.LEFT;
    		else if (dir == Direction.LEFT)
    			this.dir = Direction.UP;

    	}
    	
        
        @Override
        protected Node clone() {
        	Node clone = null;
            try{
                clone = (Node) super.clone();
               
            }catch(CloneNotSupportedException e){
                throw new RuntimeException(e); // won't happen
            }
           
            return clone;
           
        }
    }