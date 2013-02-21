package document;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import document.actions.CompileAction;
import document.actions.ExportTasksAction;
import document.actions.ModifyAction;
import document.actions.NewWindowAction;
import document.actions.OpenFileAction;
import document.actions.OpenTaskDescriptionEditorAction;
import document.actions.PointAction;
import document.actions.RedoAction;
import document.actions.RemoveAction;
import document.actions.RunAction;
import document.actions.SaveImageAction;
import document.actions.SaveXMLAction;
import document.actions.ToolAction;
import document.actions.UndoAction;
import elements.Arrow;
import elements.Decorator;
import elements.GElement;
import elements.Joint;
import elements.tasks.Task;
import elements.tasks.TaskCreator;

public class Toolbar extends JPanel {
	
	private static final long serialVersionUID = 850819550411875827L;

	private Document document = null;
	private JLabel _statusBarLabel;
	public BTDesigner designer = null;
	private JButton _undoButton;
	private JButton _redoButton;

	public ArrayList<GElement.Creator> creators = new ArrayList<GElement.Creator>();

	static public final String TIP_move = "Select and move element.";
	static public final String TIP_remove = "Select element for remove it from document";
	static public final String TIP_removeSubtree = "Select element for remove it and all its children from document";
	static public final String TIP_copy = "Select element for clone it with all its children.";
	static public final String TIP_reconnect = "Select arrow and then to tasks for reconnect.";
	static public final String TIP_modify = "Select element for modification (change text, type, etc)";

	public void setUndoButtonState(boolean enabled) {
		_undoButton.setEnabled(enabled);
	}
	
	public void setRedoButtonState(boolean enabled) {
		_redoButton.setEnabled(enabled);
	}
	
	public static void getSaveSnapShot(Component component, String fileName)
			throws Exception {
		BufferedImage img = getScreenShot(component);
		// write the captured image as a PNG
		ImageIO.write(img, "png", new File(fileName));
	}

	public static BufferedImage getScreenShot(Component component) {

		BufferedImage image = new BufferedImage(component.getWidth(),
				component.getHeight(), BufferedImage.TYPE_INT_RGB);
		// paints into image's Graphics
		component.paint(image.getGraphics());
		return image;
	}

	private JLabel tip = new JLabel(TIP_move);

	public Toolbar(BTDesigner designer) {
		this.designer = designer;
		// document = doc;
		// document.tip = tip;

		setBorder(new TitledBorder("Toolbar"));
		setPreferredSize(new Dimension(10, 65));

		setLayout(new BorderLayout());

		this.creators.add(new TaskCreator());
		this.creators.add(new Arrow.Creator());
		this.creators.add(new Decorator.Creator());
		this.creators.add(new Joint.Creator());

		JPanel buttons = new JPanel();
		buttons.setLayout(new FlowLayout(FlowLayout.LEFT, 5, 0));
		add(buttons, BorderLayout.CENTER);
		JPanel tippnl = new JPanel();
		tippnl.setLayout(new BorderLayout());
		add(tippnl, BorderLayout.SOUTH);

		JPanel pnl = new JPanel();

		// pnl.setPreferredSize(new Dimension(15,0));
		// buttons.add(pnl);

		JButton btn = new JButton();
		
		btn = new JButton();
		btn.setToolTipText("New document");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/new_tab.png")));
		btn.addActionListener(new NewWindowAction(designer));
		buttons.add(btn);
		
		btn = new JButton();
		btn.setToolTipText("Open plan");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/open.png")));
		btn.addActionListener(new OpenFileAction(designer));
		buttons.add(btn);
		
		btn = new JButton();
		btn.setToolTipText("Save plan");
		btn.setActionCommand("file_save");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/save.png")));
		btn.addActionListener(new SaveXMLAction(designer));
		buttons.add(btn);
		
		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(10, 0));
		buttons.add(pnl);
		
		_undoButton = new JButton();
		_undoButton.setToolTipText("Undo");
		_undoButton.addActionListener(new UndoAction(designer));
		_undoButton.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/undo.png")));
		buttons.add(_undoButton);

		_redoButton = new JButton();
		_redoButton.setToolTipText("Redo");
		_redoButton.addActionListener(new RedoAction(designer));
		_redoButton.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/redo.png")));
		buttons.add(_redoButton);
		
		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(10, 0));
		buttons.add(pnl);
		
		btn = new JButton();
		btn.setToolTipText("Save image");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/save_image.png")));
		btn.addActionListener(new SaveImageAction(designer));
		buttons.add(btn);
		
		btn = new JButton();
		btn.setToolTipText("Compile");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/compile.png")));
		btn.addActionListener(new CompileAction(designer));
		buttons.add(btn);
		
		btn = new JButton();
		btn.setToolTipText("Play");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/play.png")));
		btn.setActionCommand("run_runresume_plan");
		btn.addActionListener(new RunAction(designer));
		buttons.add(btn);
		
		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(10, 0));
		buttons.add(pnl);

		btn = new JButton();
		btn.setToolTipText("Remove element");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/remove.png")));
		btn.addActionListener(new RemoveAction(designer));
		buttons.add(btn);
		
		btn = new JButton();
		btn.setToolTipText("Modify element");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/modify.png")));
		btn.addActionListener(new ModifyAction(designer));
		buttons.add(btn);
		
		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(10, 0));
		buttons.add(pnl);
		
		btn = new JButton();
		btn.setToolTipText("Move");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/move.png")));
		btn.addActionListener(new PointAction(designer));
		buttons.add(btn);
		
//		for (GElement.Creator c : this.creators) {
//			btn = new JButton();
//			btn.setText(c.getToolbarName());
//			btn.addActionListener(new ToolAction(designer, c));
//			buttons.add(btn);
//		}
		
		// **************************************
		// *** Creators
		// **************************************
		
		GElement.Creator c;
		
		// Task
		c = creators.get(0);
		btn = new JButton();
		btn.setToolTipText(c.getToolbarName());
		btn.addActionListener(new ToolAction(designer, c));
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/task.png")));
		buttons.add(btn);
		
		// Arrow
		c = creators.get(1);
		btn = new JButton();
		btn.setToolTipText(c.getToolbarName());
		btn.addActionListener(new ToolAction(designer, c));
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/arrow.png")));
		buttons.add(btn);
		
		// Decorator
		c = creators.get(2);
		btn = new JButton();
		btn.setToolTipText(c.getToolbarName());
		btn.addActionListener(new ToolAction(designer, c));
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/decorator.png")));
		buttons.add(btn);
		
		// Joint
		c = creators.get(3);
		btn = new JButton();
		btn.setToolTipText(c.getToolbarName());
		btn.addActionListener(new ToolAction(designer, c));
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/joint.png")));
		buttons.add(btn);
		

		pnl = new JPanel();
		pnl.setPreferredSize(new Dimension(15, 0));
		buttons.add(pnl);

		btn = new JButton("");
		btn.setToolTipText("Export tasks");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/export.png")));
		btn.addActionListener(new ExportTasksAction(designer));
		buttons.add(btn);

		btn = new JButton("");
		btn.setToolTipText("Task description editor");
		btn.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/editor.png")));
		btn.addActionListener(new OpenTaskDescriptionEditorAction(designer));
		buttons.add(btn);

		// tippnl.add(this.tip, BorderLayout.CENTER);
		// JLabel version = new JLabel("v."+VERSION);
		// version.setFont(new Font("Arial", 1, 10));
		// buttons.add(version);
	}

	public void setActiveDocument(Document doc) {
		this.document = doc;// designer.getActiveTab().doc;
		this.document.tip = this._statusBarLabel;
	}

	// public class ToolAction implements ActionListener {
	// public ToolAction(GElement.Creator c){ this.c=c; }
	// GElement.Creator c = null;
	// public void actionPerformed(ActionEvent a) {
	// document.toolSelectionClean();
	// document.creator = c;
	// tip.setText(c.toolTip());
	// }
	// }
	// public class OpenAction implements ActionListener {
	// public void actionPerformed(ActionEvent a) {
	//
	//
	// JFileChooser fc = new JFileChooser(new File("."));
	//
	// // Show open dialog; this method does not return until the dialog is
	// closed
	// fc.showOpenDialog(Toolbar.this);
	// File selFile = fc.getSelectedFile();
	// document.loadPlan(selFile.getAbsolutePath());
	//
	// }
	// }

	// public class ImageAction implements ActionListener {
	// public void actionPerformed(ActionEvent a) {
	// FileDialog fileDialog = new FileDialog(new Frame(), "Save",
	// FileDialog.SAVE);
	// fileDialog.setFilenameFilter(new FilenameFilter() {
	// @Override
	// public boolean accept(File dir, String name) {
	// return name.endsWith(".png");
	// }
	// });
	// fileDialog.setFile("plan.png");
	// fileDialog.setVisible(true);

	// try {
	//
	// getSaveSnapShot( document,fileDialog.getFile());
	// } catch (Exception e) {
	// // TODO Auto-generated catch block
	// e.printStackTrace();
	// }
	//
	// }
	// }

	public void setTipLabel(JLabel lbl) {
		this._statusBarLabel = lbl;
	}

	public void setTipText(String msg) {
		this.tip.setText(msg);

		if (this._statusBarLabel != null)
			this._statusBarLabel.setText(msg);
	}

}
