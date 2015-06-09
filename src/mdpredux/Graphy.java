package mdpredux;

import java.awt.*;

import javax.swing.*;

public class Graphy {
	JFrame frame;
	public Graphy(String[] labels){
		this.frame = new JFrame("Object Distribution");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		JPanel contentPane = new JPanel(new GridLayout(52, labels.length));
		frame.setContentPane(contentPane);
		double[] x = new double[labels.length];
		java.util.Arrays.fill(x, 1.0/labels.length);
		paint(labels, x);
		frame.pack();
		frame.setVisible(true);
	}
	public void paint(String[] labels, double[] probs){
		JPanel toadd;
		frame.getContentPane().removeAll();
		for(int k=0; k<labels.length; k++){
			//frame.getContentPane().add(new JLabel(Double.toString(probs[k])));
			frame.getContentPane().add(new JLabel(String.format("%.5g%n", probs[k])));
		}
		for(int j = 0; j<50; j++){
			for(int i = 0; i<labels.length; i++){
				toadd = new JPanel();
				if(0.98-(j*.02) <= probs[i]){
					toadd.setBackground(Color.BLUE);
				}
				frame.getContentPane().add(toadd);
			}
		}
		for(int i = 0; i<labels.length; i++){
			frame.getContentPane().add(new JLabel(labels[i]));
		}
		frame.pack();
		frame.repaint();
	}
	
	public static void main(String[] args){
		System.out.println("DERRRRP");
		String[] objects =  {"metal_bowl", "silver_spoon", "plastic_spoon", "color_bowl"};
		new Graphy(objects);
	}
}
