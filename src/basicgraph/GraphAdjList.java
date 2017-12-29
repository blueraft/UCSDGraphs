package basicgraph;

import util.GraphLoader;

import java.util.*;

/** A class that implements a directed graph. 
 * The graph may have self-loops, parallel edges. 
 * Vertices are labeled by integers 0 .. n-1
 * and may also have String labels.
 * The edges of the graph are not labeled.
 * Representation of edges via adjacency lists.
 * 
 * @author UCSD MOOC development team and YOU
 *
 */
public class GraphAdjList extends Graph {


	private Map<Integer,ArrayList<Integer>> adjListsMap;
	
	/** 
	 * Create a new empty Graph
	 */
	public GraphAdjList () {
		adjListsMap = new HashMap<Integer,ArrayList<Integer>>();
	}

	/** 
	 * Implement the abstract method for adding a vertex. 
	 */
	public void implementAddVertex() {
		int v = getNumVertices();
		ArrayList<Integer> neighbors = new ArrayList<Integer>();
		adjListsMap.put(v,  neighbors);
	}
	
	/** 
	 * Implement the abstract method for adding an edge.
	 * @param v the index of the start point for the edge.
	 * @param w the index of the end point for the edge.  
	 */
	public void implementAddEdge(int v, int w) {
		(adjListsMap.get(v)).add(w);

	}
	
	/** 
	 * Implement the abstract method for finding all 
	 * out-neighbors of a vertex.
	 * If there are multiple edges between the vertex
	 * and one of its out-neighbors, this neighbor
	 * appears once in the list for each of these edges.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */	
	public List<Integer> getNeighbors(int v) {
		return new ArrayList<Integer>(adjListsMap.get(v));
	}

	/** 
	 * Implement the abstract method for finding all 
	 * in-neighbors of a vertex.
	 * If there are multiple edges from another vertex
	 * to this one, the neighbor
	 * appears once in the list for each of these edges.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */	
	public List<Integer> getInNeighbors(int v) {
		List<Integer> inNeighbors = new ArrayList<Integer>();
		for (int u : adjListsMap.keySet()) {
			//iterate through all edges in u's adjacency list and 
			//add u to the inNeighbor list of v whenever an edge
			//with startpoint u has endpoint v.
			for (int w : adjListsMap.get(u)) {
				if (v == w) {
					inNeighbors.add(u);
				}
			}
		}
		return inNeighbors;
	}
	 

	/** 
	 * Implement the abstract method for finding all 
	 * vertices reachable by two hops from v.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */		
	 public List<Integer> getDistance2(int v) {
         List<Integer> distance2 = new ArrayList<>();
         List<Integer> distance1 = getNeighbors(v);

		 for (int i = 0; i < distance1.size(); i++){
		     List<Integer> neighbhors = getNeighbors(distance1.get(i));
             for (int j = 0; j < neighbhors.size(); j++){
                 int vertex = neighbhors.get(j);
                 distance2.add(vertex);
             }
         }

         return distance2;
	}
	
	/**
	 * Generate string representation of adjacency list
	 * @return the String
	 */
	public String adjacencyString() {
		String s = "Adjacency list";
		s += " (size " + getNumVertices() + "+" + getNumEdges() + " integers):";

		for (int v : adjListsMap.keySet()) {
			s += "\n\t"+v+": ";
			for (int w : adjListsMap.get(v)) {
				s += w+", ";
			}
		}
		return s;
	}

    public static void main (String[] args) {
        GraphLoader.createIntersectionsFile("data/maps/ucsd.map", "data/intersections/ucsd.intersections");


        // For testing of Part 1 functionality
        // Add your tests here to make sure your degreeSequence method is returning
        // the correct list, after examining the graphs.
        System.out.println("Loading graphs based on real data...");
        System.out.println("Goal: use degree sequence to analyse graphs.");

        System.out.println("****");
        System.out.println("Roads / intersections:");
        GraphAdjList graphFromFile = new GraphAdjList();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", graphFromFile);
        System.out.println(graphFromFile);

        System.out.println("Observe all degrees are <= 12.");
        System.out.println("****");

        System.out.println(graphFromFile.getDistance2(3));
        System.out.println("\n****");

        // You can test with real road data here.  Use the data files in data/maps

//        System.out.println("Flight data:");
//        GraphAdjList airportGraph = new GraphAdjList();
//        GraphLoader.loadRoutes("data/airports/routesUA.dat", airportGraph);
//        System.out.println(airportGraph);
//        System.out.println("Observe most degrees are small (1-30), eight are over 100.");
//        System.out.println("****");
//
//        //For testing Part 2 functionality
//        // Test your distance2 code here.
//        System.out.println("Testing distance-two methods on sample graphs...");
//        System.out.println("Goal: implement method using two approaches.");


    }

    }
