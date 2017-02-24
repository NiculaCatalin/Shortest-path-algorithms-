import java.util.ArrayList;
import java.util.List;

public class GraphNode {

	List<GraphEdge> edgeList;
	
	public GraphNode(){
		edgeList=new ArrayList<GraphEdge>();
	}
	public int heuristic(){
		int min = 0;
		for(GraphEdge currentEdge : edgeList){
			if(currentEdge.cost < min)
				min = currentEdge.cost;
		}
		return min;
	}
}
