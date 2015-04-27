package gesturesubscribers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

import ros.RosBridge;
import ros.RosListenDelegate;

public class TFSubscriber implements RosListenDelegate{
	Map<String, TFNode> nodes = new ConcurrentHashMap<String, TFNode>();
	Map<Object, Object> time_stamp;
	@SuppressWarnings("unchecked")
	@Override
	public void receive(Map<String, Object> data, String stringRep) {
		List<Map<Object, Object>> transform_list = (List<Map<Object, Object>>) ((Map<Object, Object>)data.get("msg")).get("transforms");
		Map<Object, Object> header;
		Map<Object, Object> transform;
		Map<Object, Double> translation;
		Map<Object, Double> rotation;
		Vector3D vec;
		Rotation rot;
		String child;
		String parent;
		for(Map<Object, Object> list_elem : transform_list){
			header = (Map<Object, Object>) list_elem.get("header");
			this.time_stamp = (Map<Object, Object>) header.get("stamp"); //this line is test
			parent = (String)header.get("frame_id");
			transform = (Map<Object, Object>) list_elem.get("transform");
			translation = (Map<Object, Double>) transform.get("translation");
			rotation = (Map<Object, Double>) transform.get("rotation");
			child = (String) list_elem.get("child_frame_id");
			vec = new Vector3D(translation.get("x"),translation.get("y"),translation.get("z"));
			rot = new Rotation( rotation.get("w"), rotation.get("x"),rotation.get("y"),rotation.get("z"), true); //w comes last?
			nodes.put(child, new TFNode(child, parent, rot, vec));
			if(child.charAt(0) == '/') nodes.put(child.substring(1), new TFNode(child, parent, rot, vec));
			else nodes.put("/" + child, new TFNode(child, parent, rot, vec));
			if(!nodes.containsKey(parent)){
				nodes.put(parent, new TFNode(parent));
				if(parent.charAt(0) == '/') nodes.put(parent.substring(1), new TFNode(parent));
				else nodes.put("/" + parent, new TFNode(parent));
			}
			//if(child.equals("/openni_rgb_frame")){ System.out.println(rotation + "     " + rot.getAngle()); System.out.println(rotation.get("x") + "," + rotation.get("y")+ "," + rotation.get("z")+ "," + rotation.get("w"));}
		}
	}

	public double[] switch_origin(String old_origin, String new_origin, double[] curr_pos){
		//System.out.println("old: " + old_origin);
		//System.out.println("new: " + new_origin);
		//Vector3D start = new Vector3D(curr_pos);
		RealMatrix start = new Array2DRowRealMatrix(4,4);
		for(int i = 0; i<4; i++){start.setEntry(i, i, 1.0);}
		LUDecomposition inversifier;
		start.setColumn(3, curr_pos);
		if(!nodes.containsKey(old_origin)) throw new RuntimeException("No such frame:" + old_origin);
		if(!nodes.containsKey(new_origin)) throw new RuntimeException("No such frame:" + new_origin);
		TFNode old_o = nodes.get(old_origin);
		TFNode new_o = nodes.get(new_origin);
		while(old_o.parent != null){
			if(!nodes.containsKey(old_o.parent)) throw new RuntimeException("No parent found for node " + old_o.id + " with parent "+ old_o.parent);
			inversifier = new LUDecomposition(old_o.transform);
			start = start.preMultiply(inversifier.getSolver().getInverse());
			old_o = nodes.get(old_o.parent);
		}
		LinkedList<TFNode> back_down = new LinkedList<TFNode>();
		while(new_o.parent!=null){
			back_down.addFirst(new_o);
			if(!nodes.containsKey(new_o.parent)) throw new RuntimeException("No parent found for node " + new_o.id + " with parent "+ new_o.parent);
			new_o = nodes.get(new_o.parent);
		}
		if(!(new_o.id.equals(old_o.id))){
			throw new RuntimeException("Queried transform between two disjoint trees");
		}
		for(TFNode next : back_down){
			 start = start.preMultiply(next.transform);
		}
		double[] toRet = {start.getColumn(3)[0], start.getColumn(3)[1], start.getColumn(3)[2]};
		return toRet;
	}

	private class TFNode{
		public String id;
		public String parent;
		public Rotation rotation_from_parent;
		public Vector3D translation_from_parent;
		public Array2DRowRealMatrix transform;
		public TFNode(String id){
			this.id = id;
			this.parent = null;
			this.rotation_from_parent = Rotation.IDENTITY;
			this.translation_from_parent = Vector3D.ZERO;
			this.transform = new Array2DRowRealMatrix(4,4);
			for(int i = 0; i<4; i++){this.transform.setEntry(i, i, 1.0);}
			double[] temp = {translation_from_parent.getX(), translation_from_parent.getY(), translation_from_parent.getZ(), 1.0};
			this.transform.setColumn(3, temp);
			Array2DRowRealMatrix temp2 = new Array2DRowRealMatrix(4,4);
			temp2.setSubMatrix(this.rotation_from_parent.getMatrix(), 0, 0);
			temp2.setEntry(3, 3, 1);
			this.transform = this.transform.multiply(temp2);
		}
		public TFNode(String id, String parent, Rotation rotation, Vector3D translation){
			this.id = id;
			this.parent = parent;
			this.rotation_from_parent = rotation;
			this.translation_from_parent = translation;
			this.transform = new Array2DRowRealMatrix(4,4);
			for(int i = 0; i<4; i++){this.transform.setEntry(i, i, 1.0);}
			double[] temp = {translation_from_parent.getX(), translation_from_parent.getY(), translation_from_parent.getZ(), 1.0};
			this.transform.setColumn(3, temp);
			Array2DRowRealMatrix temp2 = new Array2DRowRealMatrix(4,4);
			temp2.setSubMatrix(this.rotation_from_parent.getMatrix(), 0, 0);
			temp2.setEntry(3, 3, 1);
			this.transform = this.transform.multiply(temp2);
		}
		@Override
		public String toString(){
			return id + ":\n\tparent: " + parent + "\n\ttransform: " + transform.toString() + "\n\trotation: " + rotation_from_parent.getAngle();
		}
	}
	public static void main(String[] args){
		RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		bridge.waitForConnection();
		TFSubscriber t = new TFSubscriber();
		bridge.subscribe("tf", "tf2_msgs/TFMessage", t);
		//while(!(t.nodes.containsKey("openni_link") && t.nodes.containsKey("reference/base"))){
		while(!t.nodes.containsKey("/reference/right_hand_camera")){
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				
			}
			//System.out.println(t.nodes.size());
		}
		/*for(String s : t.nodes.keySet()){
			System.out.println(s);
		}*/
		double[] derp = {0,0,0,1};
		int i = 0;
		double[] left_hand;
		while(true){
			HashMap<String, Object> data = new HashMap<String, Object>();
			HashMap<String, Object> header = new HashMap<String, Object>();
			HashMap<String, Object> point = new HashMap<String, Object>();
			HashMap<String, Object> time_stamp = new HashMap<String, Object>();
			data.put("header", header);
			data.put("point", point);
			header.put("frame_id", "reference/base");
			time_stamp.put("sec", t.time_stamp.get("sec"));
			time_stamp.put("nsec", t.time_stamp.get("nsec"));
			header.put("stamp", time_stamp);
			header.put("seq", i); i++;
			left_hand = t.switch_origin("/reference/right_hand_camera", "/reference/base", derp);
			point.put("x", left_hand[0]);
			point.put("y", left_hand[1]);
			point.put("z", left_hand[2]);
			bridge.publish("/left_hand_from_me", "geometry_msgs/PointStamped", data);	
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				
			}
		}
		//System.out.println(t.nodes.get("openni_link").toString());
		//System.out.println(t.nodes.get("openni_rgb_frame").toString());
		//System.out.println(new Vector3D(t.switch_origin("/openni_link", "/openni_rgb_frame", derp)).toString());
		//System.out.println(new Vector3D(t.switch_origin("/openni_rgb_frame", "/openni_link", derp)).toString());
		//System.out.println(new Vector3D(t.switch_origin("/reference/base", "/reference/right_hand_camera", derp)).toString());
	}
}
