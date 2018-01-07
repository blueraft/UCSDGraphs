/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	private HashMap<GeographicPoint,Node> vertices;
	private Integer numEdges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		numEdges = 0;
	    vertices = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
	    HashSet<GeographicPoint> geoVertices = new HashSet();
        geoVertices.addAll(vertices.keySet());
		return geoVertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (!(vertices.keySet().contains(location))){
            vertices.put(location,new Node(location));
            return true;
        }
		return false;
	}

	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

        if (vertices.containsKey(from) && vertices.containsKey(to)) {
            if (vertices.get(from).addEdge(vertices.get(to), roadName, roadType, length)) {
                numEdges++;
            }
        } else {
            throw new IllegalArgumentException();
        }

	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
	    if (!(vertices.containsKey(start) && vertices.containsKey(goal))) return null;
		LinkedList<Node> queue = new LinkedList();
		HashSet<Node> visited = new HashSet();
		HashMap<Node,Node> parent = new HashMap();
		Node startNode = vertices.get(start);
		Node goalNode = vertices.get(goal);
		queue.add(startNode);
		visited.add(startNode);
		parent.put(startNode, null);

		while (!queue.isEmpty()) {
		    Node current = queue.removeFirst();
		    if (current.equals(goalNode)) {
		        return pathMapper(parent,current,startNode);
            }
		    for (Node neighbhor: current.getEdges()){
		        if(!visited.contains(neighbhor)) {
                    visited.add(neighbhor);
                    parent.put(neighbhor, current);
                    queue.addLast(neighbhor);
                }
            }
            nodeSearched.accept(current.getLocation());

        }
		// Hook for visualization.  See writeup.


		return null;
	}

	private List<GeographicPoint> pathMapper(HashMap<Node,Node> parents,
                                             Node current, Node start) {
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        while (!current.equals(start)) {
            path.addFirst(current.getLocation());
            current = parents.get(current);
        }

        // add start
        path.addFirst(start.getLocation());
        return path;
    }

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
        if (!(vertices.containsKey(start) && vertices.containsKey(goal))) return null;
        HashSet<Node> visited = new HashSet();
        HashMap<Node,Node> parent = new HashMap();
        Node startNode = vertices.get(start);
        Node goalNode = vertices.get(goal);
        Comparator<Node> distanceCompare = Comparator.comparingDouble(node -> node.getPriorityDistance());
        PriorityQueue<Node> queue = new PriorityQueue<>(distanceCompare);
        Node current;
        startNode.setPriorityDistance(0);
        queue.add(startNode);
        parent.put(startNode,null);
        while (queue.size() != 0)
        {
            current = queue.remove();
            if (!visited.contains(current)){
                visited.add(current);
                if (current.equals(goalNode)) return pathMapper(parent,current,startNode);
                for (Node neighbor: current.getEdges()){
                    double distance = current.getPriorityDistance() + neighbor.getDistance(current);
                    if (!visited.contains(neighbor) && neighbor.getPriorityDistance() > distance){
                        neighbor.setPriorityDistance(distance);
                        parent.put(neighbor, current);
                        queue.add(neighbor);
                    }
                }
            }
        }
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

    private class Node {
        private GeographicPoint location;
        private HashMap<Node,ArrayList> edges;
        private double priorityDistance;

	    Node(GeographicPoint location){
	        this.location = location;
	        edges = new HashMap<>();
	        priorityDistance = Double.POSITIVE_INFINITY;

        }

        private GeographicPoint getLocation() {
	        return location;
        }

        private List<Node> getEdges(){
            List<Node> edgelist = new ArrayList();
            edgelist.addAll(edges.keySet());
            return edgelist;
        }

        private double getDistance(Node edge){
            if (!edges.containsKey(edge)) return 0;
            return (double) edges.get(edge).get(2);
        }

        private void setPriorityDistance(double pDistance){
            priorityDistance = pDistance;
        }

        private double getPriorityDistance(){
            return priorityDistance;
        }
        private boolean addEdge(Node end,String roadName,
                        String roadType, double length){
	        if (!edges.containsKey(end)) {
	            ArrayList list = new ArrayList();
	            list.add(roadName);
	            list.add(roadType);
	            list.add(length);
	            edges.put(end,list);
                return true;
            }
	        return false;
        }

    }
	
	public static void main(String[] args)
	{

        MapGraph simpleTestMap = new MapGraph();
        GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
        GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
        simpleTestMap.addVertex(testEnd);
        simpleTestMap.addVertex(testStart);
        simpleTestMap.addEdge(testStart,testEnd,"test","fsf",12.3);
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
        List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
        System.out.println(testroute);


//
//
//        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//        List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//        List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}

	    /* with help from stackoverflow link below
		 * https://stackoverflow.com/questions/683041/how-do-i-use-a-priorityqueue
		 */


}

