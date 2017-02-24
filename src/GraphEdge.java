
public class GraphEdge implements Comparable<GraphEdge>{
	int cost;
	int sourceNode;
	int destinationNode;
	int heuristic=0;

	public GraphEdge(int cost, int sourceNode, int destinationNode) {
		
		this.cost = cost;
		this.sourceNode = sourceNode;
		this.destinationNode = destinationNode;
	}
	

	@Override
	public int compareTo(GraphEdge arg0) {
		return cost + heuristic - arg0.cost - arg0.heuristic;
	}
}
