package document;

import javax.swing.ImageIcon;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JSeparator;

import document.BTDesigner.DesignerTab;

public class DesignerPopupMenu extends JPopupMenu {

	private static final long serialVersionUID = 1698952859660090169L;

	private BTDesigner _designer;
	private Document _document; 
	
	private JMenu createCopyToPopup() {
		JMenu menu = new JMenu("Copy to...");
		
		for (DesignerTab tab : _designer.tabs) {
			JMenuItem item = new JMenuItem(tab.doc.getShortFilePath());
			item.setEnabled(!tab.doc.getShortFilePath().equals(_document.getShortFilePath()));
			item.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/open.png")));
			menu.add(item);
		}
		
		return menu;
	}
	
	public DesignerPopupMenu(BTDesigner designer, Document document) {
		_designer = designer;
		_document = document;
		
		// JMenuItem item = new JMenuItem("Copy to...");
		
		add(new JMenuItem("Modify"));
		add(new JMenuItem("Remove"));
		add(new JSeparator());
		add(createCopyToPopup());
	}
	
}
