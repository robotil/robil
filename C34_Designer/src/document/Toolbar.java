package document;

import document.actions.*;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FileDialog;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.border.TitledBorder;


import elements.Arrow;
import elements.Joint;
import elements.Decorator;
import elements.GElement;
import elements.Arrow.Creator;
import elements.Modifier;
import elements.Task;

public class Toolbar extends JPanel {
	{
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InstantiationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedLookAndFeelException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private Document document = null;
	public BTDesigner designer = null;
	
	public ArrayList<GElement.Creator> creators = new ArrayList<GElement.Creator>();


	static public final String TIP_move = "Select and move element.";
	static public final String TIP_remove = "Select element for remove it from document";
	static public final String TIP_modify = "Select element for modification (change text, type, etc)";
	public JLabel tip = new JLabel(TIP_move);

	public void setTipText(String msg) {
		tip.setText(msg);
	}

	public void setActiveDocument(Document doc) {
		document = doc;//designer.getActiveTab().doc;
		document.tip = tip;
	}
	
	public Toolbar(BTDesigner designer){
		this.designer = designer;
//		document = doc;
//		document.tip = tip;

		
		setBorder(new TitledBorder("Toolbar"));
		setPreferredSize(new Dimension(10,75));

		setLayout(new BorderLayout());

		creators.add(new Task.Creator());
		creators.add(new Arrow.Creator());
		creators.add(new Decorator.Creator());
		creators.add(new Joint.Creator());

		JPanel buttons = new JPanel();
		buttons.setLayout(new FlowLayout(FlowLayout.LEFT,5,0));
		add(buttons, BorderLayout.CENTER);
		JPanel tippnl = new JPanel();
		tippnl.setLayout(new BorderLayout());
		add(tippnl, BorderLayout.SOUTH);


		JPanel pnl = new JPanel();

		pnl.setPreferredSize(new Dimension(15,0));
		buttons.add(pnl);

		JButton btn = new JButton();
		btn.setText("Open");
		btn.addActionListener(new OpenFileAction(designer));
		buttons.add(btn);
		btn = new JButton();
		btn.setText("Image");
		btn.addActionListener(new SaveImageAction(designer));
		buttons.add(btn);
		btn = new JButton();
		btn.setText("Compile");
		btn.addActionListener(new CompileAction(designer));
		buttons.add(btn);
		btn = new JButton();
		btn.setText("Run");
		btn.setActionCommand("run_run_plan");
		btn.addActionListener(new RunAction(designer));
		buttons.add(btn);
		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(15,0));
		buttons.add(pnl);
		
		btn = new JButton();
		btn.setText("Remove");
		btn.addActionListener(new RemoveAction(designer));
		buttons.add(btn);
		btn = new JButton();
		btn.setText("Modify");
		btn.addActionListener(new ModifyAction(designer));
		buttons.add(btn);
		btn = new JButton();
		btn.setText("Move");
		btn.addActionListener(new PointAction(designer));
		buttons.add(btn);
		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(15,0));
		buttons.add(pnl);

		for(GElement.Creator c : creators){
			btn = new JButton();
			btn.setText(c.getToolbarName());
			btn.addActionListener(new ToolAction(designer, c));
			buttons.add(btn);
		}

		tippnl.add(tip, BorderLayout.CENTER);
//		JLabel version = new JLabel("v."+VERSION);
//		version.setFont(new Font("Arial", 1, 10));
//		buttons.add(version);
	}


//	public class ToolAction implements ActionListener {
//		public ToolAction(GElement.Creator c){ this.c=c; }
//		GElement.Creator c = null;
//		public void actionPerformed(ActionEvent a) {
//			document.toolSelectionClean();
//			document.creator = c;
//			tip.setText(c.toolTip());
//		}	
//	}
//	public class OpenAction implements ActionListener {
//		public void actionPerformed(ActionEvent a) {
//
//
//			JFileChooser fc = new JFileChooser(new File("."));
//
//			// Show open dialog; this method does not return until the dialog is closed
//			fc.showOpenDialog(Toolbar.this);
//			File selFile = fc.getSelectedFile();
//			document.loadPlan(selFile.getAbsolutePath());
//			
//		}	
//	}

//	public class ImageAction implements ActionListener {
//		public void actionPerformed(ActionEvent a) {
//			FileDialog fileDialog = new FileDialog(new Frame(), "Save", FileDialog.SAVE);
//			fileDialog.setFilenameFilter(new FilenameFilter() {
//				@Override
//				public boolean accept(File dir, String name) {
//					return name.endsWith(".png");
//				}
//			});
//			fileDialog.setFile("plan.png");
//			fileDialog.setVisible(true);
//			System.out.println("File: " + fileDialog.getFile());
//			try {
//				
//				getSaveSnapShot( document,fileDialog.getFile());
//			} catch (Exception e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//
//		}	
//	}

	public static BufferedImage getScreenShot(Component component) {

		BufferedImage image = new BufferedImage(component.getWidth(), component.getHeight(), BufferedImage.TYPE_INT_RGB);
		// paints into image's Graphics
		component.paint(image.getGraphics());
		return image;
	}

	public static void getSaveSnapShot(Component component, String fileName) throws Exception {
		BufferedImage img = getScreenShot(component);
		// write the captured image as a PNG
		ImageIO.write(img, "png", new File(fileName));
	}

}
