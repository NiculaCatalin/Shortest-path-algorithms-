import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Scanner;

import javax.print.attribute.standard.NumberOfDocuments;

public class Main {
	static GraphNode[] graph;
	static int numberOfNodes;
	static int numberOfEdges;
	static int cost;
	static int Edges;

	public static void read() {
		try {
			Scanner information = new Scanner(new File("Graph.in.txt"));
			int source, destination, cost;
			GraphEdge currentEdge;
			numberOfNodes = information.nextInt();
			numberOfEdges = information.nextInt();
			graph = new GraphNode[numberOfNodes + 1];
			for (int i = 0; i < numberOfNodes; i++) {
				graph[i] = new GraphNode();
			}
			for (int i = 0; i < numberOfEdges; i++) {
				source = information.nextInt();
				destination = information.nextInt();
				cost = information.nextInt();
				currentEdge = new GraphEdge(cost, source, destination);
				graph[source].edgeList.add(currentEdge);

			}
			information.close();
		} catch (FileNotFoundException exception) {
			exception.printStackTrace();
		}

	}

	public static void BFS(int sourceNode, int destinationNode) {
		int currentNode = sourceNode;
		int beenThereDoneThat[] = new int[numberOfEdges];
		Edges = 0;
		beenThereDoneThat[currentNode] = 1;
		PriorityQueue<GraphEdge> queue = new PriorityQueue();
		cost = 0;
		GraphEdge current;
		while (destinationNode != currentNode) {
			for (GraphEdge currentEdge : graph[currentNode].edgeList) {
				if (beenThereDoneThat[currentEdge.destinationNode] != 1) {
					queue.add(currentEdge);
				}
				Edges ++;
			}
			current = queue.poll();
			beenThereDoneThat[current.destinationNode]=1;
			cost = cost + current.cost;
			currentNode = current.destinationNode;

		}
	}

	public static void Dijkstra(int sourceNode, int destinationNode) {
		int currentNode = sourceNode;
		int costVector[] = new int[numberOfEdges];
		GraphEdge current;
		cost = 0;
		costVector[currentNode] = 0;
		Edges = 0;
		PriorityQueue<GraphEdge> queue = new PriorityQueue();
		queue.add(graph[currentNode].edgeList.get(0));
		while (!queue.isEmpty()) {
			for (GraphEdge currentEdge : graph[currentNode].edgeList) {
				Edges++;
				int newCost=costVector[currentNode]+currentEdge.cost;
				if (costVector[currentEdge.destinationNode]==0 || costVector[currentEdge.destinationNode]>newCost){
					queue.add(currentEdge);
					costVector[currentEdge.destinationNode]= newCost;	
				}
				
				
			}
			
			current = queue.poll();
			
			currentNode = current.destinationNode;
			cost = costVector[destinationNode];

		}
		costVector[0]=0;
	}
	public static void AStar(int sourceNode, int destinationNode) {
		int currentNode = sourceNode;
		int[] heuristics = new int[numberOfEdges];
		int min;
		for(int i=0;i<numberOfEdges; i++){
			min = 0;
			for(GraphEdge currentEdge : graph[currentNode].edgeList){
				if(min> currentEdge.cost)
					min = currentEdge.cost;
			heuristics[i]=min;
			}
		}
		GraphEdge current;
		cost = 0;
		PriorityQueue<GraphEdge> queue = new PriorityQueue();
		while (destinationNode != currentNode) {
			for (GraphEdge currentEdge : graph[currentNode].edgeList) {
				
				currentEdge.heuristic = heuristics[currentEdge.destinationNode] + 
						current.heuristic - heuristics[current.destinationNode] + current.cost;
					queue.add(currentEdge);
				
			}
			current = queue.poll();
			cost = cost + current.cost;
			currentNode = current.destinationNode;

		}
	}

	public static void main(String[] args) {
		
		read();
		long startTime = System.nanoTime(), bfsTime, dijkTime, asTime;
		BFS(0, 5);
		bfsTime = System.nanoTime();
		System.out.println(cost);
		
		System.out.println("Run time: " + (bfsTime - startTime) / 1000 + " miliseconds");
		System.out.println(Edges);
		dijkTime = System.nanoTime();
		Dijkstra(0, 5);
		System.out.println(cost);
		System.out.println("Run time: " + (dijkTime - bfsTime) / 1000 + " miliseconds");
		System.out.println(Edges);
		
		asTime = System.nanoTime();
		AStar(0, 5);
		System.out.println(cost);
		System.out.println("Run time: " + (asTime - bfsTime) / 1000 + " miliseconds");
		System.out.println(Edges);
		
	}

}
