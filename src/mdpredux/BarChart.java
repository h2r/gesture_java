package mdpredux;
import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.renderer.category.BarRenderer;
import org.jfree.data.category.DefaultCategoryDataset;

import java.awt.*;

/**
 * Created by dwhitney on 6/3/15.
 */
public class BarChart extends JFrame{

    private static final long serialVersionUID = 1L;
    private DefaultCategoryDataset dataset;
    private String chartTitle;

    public BarChart(String applicationTitle, String chartTitle) {
        super(applicationTitle);
        this.chartTitle = chartTitle;
        this.dataset = new DefaultCategoryDataset();
    }

    public void draw() {
        // based on the dataset we create the chart
        JFreeChart chart = ChartFactory.createBarChart3D(chartTitle, "Category", "Score", dataset, PlotOrientation.VERTICAL, true, true, false);
        CategoryPlot cPlot = (CategoryPlot) chart.getPlot();
        NumberAxis range = (NumberAxis) cPlot.getRangeAxis();
        BarRenderer renderer = (BarRenderer) cPlot.getRenderer();
        renderer.setSeriesPaint(0, Color.GREEN);
        range.setRange(0.0,1.0);
        // Adding chart into a chart panel
        ChartPanel chartPanel = new ChartPanel(chart);

        // settind default size
        chartPanel.setPreferredSize(new java.awt.Dimension(800, 570));

        // add to contentPane
        setContentPane(chartPanel);

        this.pack();
        this.setVisible(true);
    }
    public void createDataset(String[] objects, double[] beliefs) {

        // row keys...
        final String prob = "Probability";

        // create the dataset...
        final DefaultCategoryDataset data = new DefaultCategoryDataset();

        for (int i = 0; i < objects.length; i++) {
            data.addValue(beliefs[i], prob, objects[i]);
        }
        this.dataset = data;
    }

    public static void main(String[] args) {
        BarChart chart = new BarChart("Robot Internal Estimate", "Object Probability Distribution");
        chart.pack();
        chart.setVisible(true);
    }
}
