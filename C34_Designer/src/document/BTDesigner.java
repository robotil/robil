package document;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;

import javax.swing.KeyStroke;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Element;
import org.xml.sax.SAXException;

public class BTDesigner extends JFrame {

	public final static String VERSION = "0.1.1";

	ArrayList<Document> documents = new ArrayList<Document>();
	Document activeDocument;
	
	public Document getActiveDocument() {
		
		if (activeDocument == null) {
			addNewDocumentTab();
			toolbar.setActiveDocument();
		}
		
		return activeDocument;
	}
	
	public int getNumberOfDocuments() {
		return documents.size();
	}

	public Toolbar toolbar;
	public JTabbedPane tabbedPane = new JTabbedPane();

	

	public BTDesigner() {

		this.setTitle("Cogniteam BTDesigner " + BTDesigner.VERSION);

		addNewDocumentTab();

		toolbar = new Toolbar(this);

		setLocation(200, 50);
		setSize(new Dimension(1000, 700));
		ImageIcon icon = new ImageIcon(getClass().getClassLoader().getResource(
				"icons/CogniTeam.gif"));
		this.setIconImage(icon.getImage());
		setLayout(new BorderLayout());

		Menubar menuBar = new Menubar(this);
		JPanel panelMenus = new JPanel(new BorderLayout());
		panelMenus.add(menuBar, BorderLayout.NORTH);
		panelMenus.add(toolbar, BorderLayout.SOUTH);
		// add(menuBar, BorderLayout.NORTH);
		add(panelMenus, BorderLayout.NORTH);


		add(tabbedPane, BorderLayout.CENTER);
		// add(toolbar, BorderLayout.SOUTH);

		// add(document, BorderLayout.CENTER);
		this.setJMenuBar(menuBar);
	}

	public void addNewDocumentTab() {
		int numOfTabs = tabbedPane.getTabCount();
		JPanel panelDoc = new JPanel(new BorderLayout());

		// add new document
		activeDocument = new Document(this);
		documents.add(activeDocument);

		panelDoc.add(activeDocument, BorderLayout.CENTER);
		tabbedPane.addTab("New", panelDoc);
		tabbedPane.setTabComponentAt(numOfTabs, new ButtonTabComponent(
				tabbedPane, this));
	}
	
	public void setTabName(int index, String name) {

		// validate index
		if (index < 0 || index >= tabbedPane.getTabCount()) {
			return;
		}

		tabbedPane.setTitleAt(index, name);
	}

	public static void main(String[] args) {
		org.w3c.dom.Document doc = null;
		try {
			doc = DocumentBuilderFactory.newInstance().newDocumentBuilder()
					.parse(new File("BTDesigner.xml"));
		} catch (javax.xml.parsers.ParserConfigurationException ex) {
			ex.printStackTrace();
		} catch (SAXException ex) {
			ex.printStackTrace();
		} catch (IOException ex) {
			ex.printStackTrace();
		}
		Element el = (Element) (doc.getElementsByTagName("test_time").item(0));
		Parameters.test_time = Integer.parseInt(el.getAttribute("value"));
		el = (Element) (doc.getElementsByTagName("test_result").item(0));
		Parameters.test_result = Boolean.parseBoolean(el.getAttribute("value"));

		BTDesigner btd = new BTDesigner();
		btd.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		btd.setVisible(true);
	}

}
