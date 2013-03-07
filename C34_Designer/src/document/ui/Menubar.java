package document.ui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;

import javax.swing.ImageIcon;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.KeyStroke;

import windows.designer.BTDesigner;

import document.actions.*;

public class Menubar extends JMenuBar {

	private JMenuItem _undoMenuItem;
	private JMenuItem _redoMenuItem;
	private JCheckBoxMenuItem _debugViewMenuItem;
	private JCheckBoxMenuItem _runtimeViewMenuItem;
	
	private static final long serialVersionUID = 8066873848483126226L;

	public Menubar(BTDesigner designer) {
		add(buildFileMenu(designer));
		add(buildEditMenu(designer));
		add(buildViewMenu(designer));
		add(buildRunMenu(designer));
		add(buildWindowMenu(designer));
	}

	private JMenu buildViewMenu(final BTDesigner designer) {
		JMenu menu = new JMenu("View");
		_debugViewMenuItem = new JCheckBoxMenuItem("Enable debug view");
		_debugViewMenuItem.setSelected(false);
		_debugViewMenuItem.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg) {
				designer.getActiveTab().doc.repaint();
				
				if (_debugViewMenuItem.isSelected())
					_runtimeViewMenuItem.setSelected(false);
			}
		});
		menu.add(_debugViewMenuItem);
		
		_runtimeViewMenuItem = new JCheckBoxMenuItem("Enable runtime view");
		_runtimeViewMenuItem.setSelected(false);
		_runtimeViewMenuItem.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg) {
				designer.getActiveTab().doc.repaint();
				
				if (_runtimeViewMenuItem.isSelected())
					_debugViewMenuItem.setSelected(false);
			}
		});
		menu.add(_runtimeViewMenuItem);
		
		return menu;
	}
	
	private JMenu buildEditMenu(BTDesigner designer) {
		JMenu menu = new JMenu("Edit");

		JMenuItem undo = new JMenuItem("Undo");
		undo.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_Z,ActionEvent.CTRL_MASK));
		undo.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/undo.png")));
		undo.addActionListener(new UndoAction(designer));
		
		JMenuItem redo = new JMenuItem("Redo");
		redo.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_R,ActionEvent.CTRL_MASK));
		redo.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/redo.png")));
		redo.addActionListener(new RedoAction(designer));
		
		menu.add(undo);
		menu.add(redo);
		
		_undoMenuItem = undo;
		_redoMenuItem = redo;
		
		menu.setMnemonic(KeyEvent.VK_E);
		menu.add(buildToolsMenu(designer));
		menu.add(buildElementsCreatorMenu(designer));

		ImageIcon icon = new ImageIcon(getClass().getClassLoader().getResource("icons/copy.png"));
		JMenuItem menuItemCopy = new JMenuItem("Copy", KeyEvent.VK_C);
		menuItemCopy.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_1, ActionEvent.ALT_MASK));
		menuItemCopy.setToolTipText("Copy selected sub-tree");
		menuItemCopy.setIcon(icon);
		menuItemCopy.addActionListener(new CopyTreeAction(designer));
		menu.add(menuItemCopy);

		return menu;
	}

	private JMenu buildElementsCreatorMenu(BTDesigner designer) {
		JMenu menu = new JMenu("Elements Creator");
		menu.setMnemonic(KeyEvent.VK_C);

		// task
		JMenuItem menuItemTask = new JMenuItem("Task", KeyEvent.VK_T);
		menuItemTask.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_1, ActionEvent.ALT_MASK));
		menuItemTask.setToolTipText("sets mode of selector to create tasks");
		menuItemTask.addActionListener(new ToolAction(designer,
				designer.toolbar.creators.get(0)));
		menu.add(menuItemTask);

		// arrow
		JMenuItem menuItemArrow = new JMenuItem("Arrow", KeyEvent.VK_A);
		menuItemArrow.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_2,
				ActionEvent.ALT_MASK));
		menuItemArrow.setToolTipText("sets mode of selector to create arrows");
		menuItemArrow.addActionListener(new ToolAction(designer,
				designer.toolbar.creators.get(1)));
		menu.add(menuItemArrow);

		// decorator
		JMenuItem menuItemDecorator = new JMenuItem("Decorator", KeyEvent.VK_D);
		menuItemDecorator.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_3,
				ActionEvent.ALT_MASK));
		menuItemDecorator
				.setToolTipText("sets mode of selector to create decorators");
		menuItemDecorator.addActionListener(new ToolAction(designer,
				designer.toolbar.creators.get(2)));
		menu.add(menuItemDecorator);

		// joint
		JMenuItem menuItemJoint = new JMenuItem("Joint", KeyEvent.VK_J);
		menuItemJoint.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_4,
				ActionEvent.ALT_MASK));
		menuItemJoint
				.setToolTipText("sets mode of selector to create joint points on arrows");
		menuItemJoint.addActionListener(new ToolAction(designer,
				designer.toolbar.creators.get(3)));
		menu.add(menuItemJoint);

		return menu;
	}

	private JMenu buildFileMenu(BTDesigner designer) {
		/* build File Menu */

		JMenu menuFile = new JMenu("File");

		ImageIcon icon;

		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/new_tab.png"));
		JMenuItem menuItemNew = new JMenuItem("New document");
		menuItemNew.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_T, ActionEvent.CTRL_MASK));
		menuItemNew.setToolTipText("Creates new instance of BTDesigner");
		menuItemNew.addActionListener(new NewWindowAction(designer));
		menuItemNew.setIcon(icon);
		menuFile.add(menuItemNew);
		
		
		menuFile.setMnemonic(KeyEvent.VK_F);

		// open file
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/open.png"));
		JMenuItem menuItemOpen = new JMenuItem("Open", KeyEvent.VK_O);
		menuItemOpen.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, ActionEvent.CTRL_MASK));
		menuItemOpen.setToolTipText("open file");
		menuItemOpen.setActionCommand("file_open");
		menuItemOpen.setIcon(icon);
		menuItemOpen.addActionListener(new OpenFileAction(designer));
		menuFile.add(menuItemOpen);

		// load and open
		icon = new ImageIcon(getClass().getClassLoader().getResource("icons/load_open.png"));
		JMenuItem menuItemLoadAndOpen = new JMenuItem("Load and Open", KeyEvent.VK_L);
		menuItemLoadAndOpen.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, ActionEvent.CTRL_MASK | ActionEvent.SHIFT_MASK));
		menuItemLoadAndOpen.setToolTipText("load and open file");
		menuItemLoadAndOpen.setActionCommand("file_load_and_open");
		menuItemLoadAndOpen.addActionListener(new LoadAndOpenAction(designer));
		menuItemLoadAndOpen.setIcon(icon);
		menuFile.add(menuItemLoadAndOpen);

		// save
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/save.png"));
		JMenuItem menuItemSave = new JMenuItem("Save", KeyEvent.VK_S);
		menuItemSave.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, ActionEvent.CTRL_MASK));
		menuItemSave.setToolTipText("save file");
		menuItemSave.setActionCommand("file_save");
		menuItemSave.setIcon(icon);
		menuItemSave.addActionListener(new SaveXMLAction(designer));
		// menuItemSave.addActionListener(listener);
		menuFile.add(menuItemSave);

		// save as
		JMenuItem menuItemSaveAs = new JMenuItem("Save As...", KeyEvent.VK_A);
		menuItemSaveAs.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, ActionEvent.CTRL_MASK | ActionEvent.SHIFT_MASK));
		menuItemSaveAs.setToolTipText("save file as");
		menuItemSaveAs.setActionCommand("file_save_as");
		menuItemSaveAs.addActionListener(new SaveXMLAction(designer));
		menuFile.add(menuItemSaveAs);

		// save image
		icon = new ImageIcon(getClass().getClassLoader().getResource("icons/save_image.png"));
		JMenuItem menuItemSaveImage = new JMenuItem("Save Image", KeyEvent.VK_I);
		menuItemSaveImage.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_I, ActionEvent.CTRL_MASK));
		menuItemSaveImage.setToolTipText("creates PNG image and saves it locally");
		menuItemSaveImage.setActionCommand("file_save_image");
		menuItemSaveImage.setIcon(icon);
		menuItemSaveImage.addActionListener(new SaveImageAction(designer));
		menuFile.add(menuItemSaveImage);

		// compile
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/compile.png"));
		JMenuItem menuItemCompile = new JMenuItem("Compile");
		menuItemCompile.setMnemonic(KeyEvent.VK_C);
		menuItemCompile.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_B, ActionEvent.CTRL_MASK));
		menuItemCompile
				.setToolTipText("validates current plan and if plan is valid, saves it");
		menuItemCompile.addActionListener(new CompileAction(designer));
		menuItemCompile.setIcon(icon);
		menuFile.add(menuItemCompile);

		// compile and upload
		JMenuItem menuItemCompileAndUpload = new JMenuItem(
				"Compile And Upload", KeyEvent.VK_U);
		menuItemCompileAndUpload.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_B, ActionEvent.CTRL_MASK | ActionEvent.SHIFT_MASK));
		menuItemCompileAndUpload
				.setToolTipText("validates current plan and if plan is valid uploads it to remote host (C34_Executer’s host)");
		// menuItemCompileAndUpload.addActionListener(listener);
		menuFile.add(menuItemCompileAndUpload);

		// Lookup table editor
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/lookup.png"));
		JMenuItem menuLookup = new JMenuItem("Lookup table editor");
		menuLookup.addActionListener(new OpenLookupTableEditorAction(designer));
		menuLookup.setIcon(icon);
		menuFile.add(menuLookup);
		
		// // run
		// icon = new ImageIcon(getClass().getClassLoader().getResource(
		// "icons/play.png"));
		// JMenuItem menuItemRun = new JMenuItem("Run", KeyEvent.VK_R);
		// menuItemRun.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_8,
		// ActionEvent.ALT_MASK));
		// menuItemRun.setIcon(icon);
		// menuItemRun
		// .setToolTipText("validates current plan, uploads it to temporal remote file on C34_Executer’s host and runs it");
		// menuItemRun.addActionListener(new RunAction(designer));
		// menuFile.add(menuItemRun);

		// properties
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/settings.png"));
		JMenuItem menuItemProperties = new JMenuItem("Properties",
				KeyEvent.VK_P);
		menuItemProperties.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_P,
				ActionEvent.CTRL_MASK));
		menuItemProperties.setToolTipText("open properties dialog");
		menuItemProperties.setActionCommand("open_properties_dialog");
		menuItemProperties.addActionListener(new PropertiesAction());
		menuItemProperties.setIcon(icon);
		menuFile.add(menuItemProperties);

		return menuFile;
	}

	private JMenu buildRunMenu(BTDesigner designer) {
		JMenu menu = new JMenu("Run");
		menu.setMnemonic(KeyEvent.VK_R);

		ImageIcon icon;
		JMenuItem menuItem;
		int shortcutKey = KeyEvent.VK_1;

		// Run plan
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/robot.png"));
		menuItem = new JMenuItem("Run", KeyEvent.VK_R);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(shortcutKey++,
				ActionEvent.ALT_MASK));
		menuItem.setToolTipText("Runs current plan");
		menuItem.addActionListener(new RunAction(designer));
		menuItem.setActionCommand("run_run_plan");
		menuItem.setIcon(icon);
		menu.add(menuItem);

		// Resume
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/play.png"));
		menuItem = new JMenuItem("Resume", KeyEvent.VK_E);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(shortcutKey++,
				ActionEvent.ALT_MASK));
		menuItem.setToolTipText("Resumes current execution");
		menuItem.addActionListener(new RunAction(designer));
		menuItem.setActionCommand("run_resume_plan");
		menuItem.setIcon(icon);
		menu.add(menuItem);

		// pause
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/pause.png"));
		menuItem = new JMenuItem("Pause", KeyEvent.VK_P);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(shortcutKey++,
				ActionEvent.ALT_MASK));
		menuItem.setToolTipText("Pauses current execution");
		menuItem.addActionListener(new RunAction(designer));
		menuItem.setActionCommand("run_pause_plan");
		menuItem.setIcon(icon);
		menu.add(menuItem);

		// step
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/step.png"));
		menuItem = new JMenuItem("Step", KeyEvent.VK_S);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(shortcutKey++,
				ActionEvent.ALT_MASK));
		menuItem.setToolTipText("Steps current execution");
		menuItem.addActionListener(new RunAction(designer));
		menuItem.setActionCommand("run_step_plan");
		menuItem.setIcon(icon);
		menu.add(menuItem);

		// stop
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/stop.png"));
		menuItem = new JMenuItem("Stop", KeyEvent.VK_O);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(shortcutKey++,
				ActionEvent.ALT_MASK));
		menuItem.setToolTipText("Stops current execution");
		menuItem.addActionListener(new RunAction(designer));
		menuItem.setActionCommand("run_stop_plan");
		menuItem.setIcon(icon);
		menu.add(menuItem);

		// test
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/test.png"));
		menuItem = new JMenuItem("Test", KeyEvent.VK_T);
		menuItem.setAccelerator(KeyStroke.getKeyStroke(shortcutKey++,
				ActionEvent.ALT_MASK));
		menuItem.setIcon(icon);
		menuItem.addActionListener(new TestAction(designer));
		menuItem.setToolTipText("validates current plan, saves it to a temporal local file and runs it in mode of logic simulation");
		// menuItemTest.addActionListener(listener);
		menu.add(menuItem);

		return menu;
	}

	private JMenu buildToolsMenu(BTDesigner designer) {
		JMenu menu = new JMenu("Tools");
		menu.setMnemonic(KeyEvent.VK_T);

		// remove
		ImageIcon icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/remove.png"));
		JMenuItem menuItemRemove = new JMenuItem("Remove", KeyEvent.VK_R);
		menuItemRemove.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_1,
				ActionEvent.ALT_MASK));
		menuItemRemove.setToolTipText("sets mode of selector to remove");
		menuItemRemove.setIcon(icon);
		menuItemRemove.addActionListener(new RemoveAction(designer));
		menu.add(menuItemRemove);

		// remove Subtree
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/remove.png"));
		JMenuItem menuItemRemoveSubtree = new JMenuItem("Remove sub-tree",
				KeyEvent.VK_T);
		menuItemRemoveSubtree.setAccelerator(KeyStroke.getKeyStroke(
				KeyEvent.VK_4, ActionEvent.ALT_MASK));
		menuItemRemoveSubtree
				.setToolTipText("sets mode of selector to remove subtree");
		menuItemRemoveSubtree.setIcon(icon);
		menuItemRemoveSubtree.addActionListener(new RemoveSubtreeAction(
				designer));
		menu.add(menuItemRemoveSubtree);

		// remove Subtree
		// icon = new ImageIcon(getClass().getClassLoader().getResource(
		// "icons/remove.png"));
		JMenuItem menuItemReconnect = new JMenuItem("Reconnect", KeyEvent.VK_C);
		menuItemReconnect.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_5,
				ActionEvent.ALT_MASK));
		menuItemReconnect.setToolTipText("arrow reconnect");
		// menuItemReconnect.setIcon(icon);
		menuItemReconnect.addActionListener(new ArrowReconnectAction(designer));
		menu.add(menuItemReconnect);

		// modify
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/modify.png"));
		JMenuItem menuItemModify = new JMenuItem("Modify", KeyEvent.VK_M);
		menuItemModify.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_2,
				ActionEvent.ALT_MASK));
		menuItemModify.setToolTipText("sets mode of selector to modify");
		menuItemModify.setIcon(icon);
		menuItemModify.addActionListener(new ModifyAction(designer));
		menu.add(menuItemModify);

		// move
		JMenuItem menuItemMove = new JMenuItem("Move", KeyEvent.VK_V);
		menuItemMove.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_3,
				ActionEvent.ALT_MASK));
		menuItemMove.setToolTipText("sets mode of selector to move");
		menuItemMove.addActionListener(new PointAction(designer));
		menu.add(menuItemMove);

		return menu;
	}

	private JMenu buildWindowMenu(BTDesigner designer) {
		JMenu menu = new JMenu("Window");
		menu.setMnemonic(KeyEvent.VK_W);

		ImageIcon icon;

		// open terminal
		icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/terminal.png"));
		JMenuItem menuItemOpenTerminal = new JMenuItem("Open Terminal",
				KeyEvent.VK_T);
		menuItemOpenTerminal.setAccelerator(KeyStroke.getKeyStroke(
				KeyEvent.VK_2, ActionEvent.ALT_MASK));
		menuItemOpenTerminal.setToolTipText("opens ROS terminal window");
		menuItemOpenTerminal.setIcon(icon);
		menuItemOpenTerminal.addActionListener(new OpenTerminalAction());
		menu.add(menuItemOpenTerminal);

		// open log console
		JMenuItem menuItemOpenLogConsole = new JMenuItem("Open Log Console",
				KeyEvent.VK_L);
		menuItemOpenLogConsole.setAccelerator(KeyStroke.getKeyStroke(
				KeyEvent.VK_3, ActionEvent.ALT_MASK));
		menuItemOpenLogConsole.setToolTipText("opens log history dialog");
		menuItemOpenLogConsole.addActionListener(new LogConsoleAction());
		menu.add(menuItemOpenLogConsole);

		// about
		JMenuItem menuItemAbout = new JMenuItem("About", KeyEvent.VK_B);
		menuItemAbout.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_4,
				ActionEvent.ALT_MASK));
		menuItemAbout.setToolTipText("About the software");
		menu.add(menuItemAbout);

		JMenuItem nextTab = new JMenuItem("Next tab");
		nextTab.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_2, ActionEvent.CTRL_MASK));
		nextTab.addActionListener(new NextTabAction(designer));
		menu.add(nextTab);
		
		JMenuItem prevTab = new JMenuItem("Previous tab");
		prevTab.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_1, ActionEvent.CTRL_MASK));
		prevTab.addActionListener(new PreviousTabAction(designer));
		menu.add(prevTab);
		
		JMenuItem closeTab = new JMenuItem("Close tab");
		closeTab.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_W, ActionEvent.CTRL_MASK));
		closeTab.addActionListener(new CloseTabAction(designer));
		menu.add(closeTab);
		
		return menu;
	}
	
	public JMenuItem getUndoButton() {
		return _undoMenuItem;
	}
	
	public JMenuItem getRedoButton() {
		return _redoMenuItem;
	}
	
	public JCheckBoxMenuItem getDebugViewMenuItem() {
		return _debugViewMenuItem;
	}
	
	public JCheckBoxMenuItem getRuntimeViewMenuItem() {
		return _runtimeViewMenuItem;
	}

	public void setRunView() {
		_runtimeViewMenuItem.setSelected(true);
		_debugViewMenuItem.setSelected(false);
	}
	
	public void setDebugView() {
		_runtimeViewMenuItem.setSelected(false);
		_debugViewMenuItem.setSelected(true);
	}

}
