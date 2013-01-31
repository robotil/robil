package document;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.util.ArrayList;
import java.util.Iterator;

import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import logger.LogManager;
import terminal.communication.RosExecutor;

public class BTDesigner extends JFrame {

	public class DesignerTab {
		public Document doc;
		public String executionID;

		public DesignerTab(Document doc, String executionID) {
			this.doc = doc;
			this.executionID = executionID;
		}

		public void clearID() {
			this.executionID = null;
		}

		public String getID() {
			return this.executionID;
		}

		public void setID(String id) {
			this.executionID = new String(id);
		}
	}

	private static final long serialVersionUID = 5495864869110385684L;

	public final static String VERSION = "0.2.0";

	public static void main(String[] args) throws Exception {

		LogManager.redirectStandardAndErrorOutput("std_err_output.txt");

		for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager
				.getInstalledLookAndFeels()) {
			if ("GTK+".equals(info.getName())) {
				try {
					javax.swing.UIManager.setLookAndFeel(info.getClassName());
				} catch (Exception e) {
					e.printStackTrace();
				}
				break;
			}
		}

		try {
			PropertiesXmlHandler.loadAndSetProperties();
		} catch (Exception e) {
			e.printStackTrace();
		}

		final BTDesigner btd = new BTDesigner();
		btd.addWindowListener(new WindowListener() {
			@Override
			public void windowClosing(WindowEvent arg0) {
				try {
					btd.close();
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
			@Override
			public void windowClosed(WindowEvent arg0) {}
			@Override
			public void windowActivated(WindowEvent arg0) {}
			@Override
			public void windowDeactivated(WindowEvent e) {}
			@Override
			public void windowDeiconified(WindowEvent e) {}
			@Override
			public void windowIconified(WindowEvent e) {}
			@Override
			public void windowOpened(WindowEvent e) {}
		});
		
		btd.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		btd.setVisible(true);
	}

	ArrayList<DesignerTab> tabs = new ArrayList<DesignerTab>();
	DesignerTab activeTab;

	public RosExecutor rosExecutor = new RosExecutor(this);

	public Toolbar toolbar;

	public JTabbedPane tabbedPane = new JTabbedPane();

	public BTDesigner() {

		this.setTitle("Cogniteam BTDesigner " + BTDesigner.VERSION);

		this.toolbar = new Toolbar(this);
		// getActiveTab();

		setLocation(200, 50);
		setSize(new Dimension(1100, 700));
		JLabel lblStatusBar = new JLabel(Toolbar.TIP_move);
		this.toolbar.setTipLabel(lblStatusBar);

		ImageIcon icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/CogniTeam.gif"));
		this.setIconImage(icon.getImage());

		// setLayout(new BorderLayout());
		GridBagConstraints c = new GridBagConstraints();
		setLayout(new GridBagLayout());

		Menubar menuBar = new Menubar(this);
		JPanel panelMenus = new JPanel(new BorderLayout());
		// panelMenus.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));
		panelMenus.add(menuBar, BorderLayout.NORTH);
		panelMenus.add(this.toolbar, BorderLayout.SOUTH);

		// add(panelMenus, BorderLayout.NORTH);
		// add(this.tabbedPane, BorderLayout.CENTER);
		c.insets = new Insets(10, 10, 0, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		c.weighty = 0.0;
		c.weightx = 1.0;
		add(panelMenus, c);

		c.insets = new Insets(10, 10, 10, 10);
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 1;
		c.weighty = 1.0;
		add(this.tabbedPane, c);

		c.insets = new Insets(0, 0, 0, 0);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 2;
		c.weighty = 0.0;

		JPanel pnl = new JPanel();
		lblStatusBar.setForeground(Color.WHITE);
		pnl.setLayout(new BorderLayout());
		pnl.setBackground(new Color(100, 100, 100));
		pnl.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
		pnl.add(lblStatusBar, BorderLayout.WEST);
		add(pnl, c);

		
		this.tabbedPane.addChangeListener(new ChangeListener() {
			
			@Override
			public void stateChanged(ChangeEvent arg0) {
				if (tabbedPane.getSelectedIndex() >= 0)
					tabs.get((tabbedPane.getSelectedIndex())).doc.activate();
			}
		});
		
		this.setJMenuBar(menuBar);

		// add new tab
		// addNewDocumentTab();
		// this.toolbar.setActiveDocument(this.tabs.get(0).doc);
	}

	public void addNewDocumentTab() {
		int numOfTabs = this.tabbedPane.getTabCount();
		JPanel panelDoc = new JPanel(new BorderLayout());

		// add new document
		this.activeTab = new DesignerTab(new Document(this), null);
		this.tabs.add(this.activeTab);

		panelDoc.add(this.activeTab.doc, BorderLayout.CENTER);
		this.tabbedPane.addTab("New", panelDoc);
		this.tabbedPane.setTabComponentAt(numOfTabs, new ButtonTabComponent(
				this.tabbedPane, this));
		
		this.tabbedPane.setSelectedIndex(this.tabbedPane.getTabCount() - 1);
	}
	
	public void addnewDocumentTab(String planFilename) {
		
		for (int i = 0; i < tabs.size(); i++) {
			if (tabs.get(i).doc.getAbsoluteFilePath().equals(planFilename)) {
				tabbedPane.setSelectedIndex(i);
				return;
			}
		}
				
		addNewDocumentTab();
		
		tabbedPane.setSelectedIndex(tabbedPane.getSelectedIndex());
		
		getActiveTab().doc.loadPlan(planFilename);
		setTabName(tabbedPane.getSelectedIndex(), getActiveTab().doc.getShortFilePath());
		// tabs.get(tabbedPane.getSelectedIndex()).doc.loadPlan(planFilename);
		
		
		// this.tabbedPane.setSelectedIndex(this.tabbedPane.getSelectedIndex());
		// this.toolbar.setActiveDocument(this.tabs.get(this.tabbedPane.getSelectedIndex()).doc);
		
		// Document document = getActiveTab().doc;
		// document.loadPlan(planFilename);
//		String shortName = document.getShortFilePath();
//		setTabName(this.tabbedPane.getSelectedIndex(), shortName);
	}

	public DesignerTab getActiveTab() {

		// if (activeTab == null) {
		//if (this.tabbedPane.getTabCount() == 0) {
		//	addNewDocumentTab();
		//}

		int index = this.tabbedPane.getSelectedIndex();

		this.activeTab = this.tabs.get(index);
		this.toolbar.setActiveDocument(this.activeTab.doc);
		return this.activeTab;
	}

	public Document getDocumentOfRunningPlan(String id) {
		for (DesignerTab tab : this.tabs)
			if (tab.getID() != null && tab.getID().equals(id))
				return tab.doc;

		return null;
	}

	public int getNumberOfDocuments() {
		return this.tabs.size();
	}

	public void setTabName(int index, String name) {

		// validate index
		if (index < 0 || index >= this.tabbedPane.getTabCount()) {
			return;
		}

		this.tabbedPane.setTitleAt(index, name);
	}

	public void close() throws Exception {
		for (DesignerTab tab : this.tabs) {
			tab.doc.close();
		}
	}

}
