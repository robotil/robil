package document;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.ImageIcon;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JSeparator;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import document.BTDesigner.DesignerTab;
import elements.GElement;

public class DesignerPopupMenu extends JPopupMenu {

	private static final long serialVersionUID = 1698952859660090169L;

	private BTDesigner _designer;
	private Document _document; 
	private GElement _element;
	
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
	
	private JCheckBoxMenuItem createCollapseCheckBox(final GElement element) {
		final JCheckBoxMenuItem menu = new JCheckBoxMenuItem("Collapsed");
		menu.setState(element.getProperty().collapsed);
		// menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/remove.png")));
		menu.addChangeListener(new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent arg) {
				element.getProperty().collapsed = menu.getState();
			}
		});
		
		return menu;
	}
	
	private JMenuItem createRemoveMenuItem(final GElement element) {
		JMenuItem menu = new JMenuItem("Remove");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/remove.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				_document.remove(element);
			}
		});
		return menu;
	}
	
	private JMenuItem createModifyMenuItem(final GElement element) {
		JMenuItem menu = new JMenuItem("Modify");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/modify.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				element.modify();
			}
		});
		return menu;
	}
	
	public DesignerPopupMenu(BTDesigner designer, Document document, GElement element) {
		_designer = designer;
		_document = document;
		_element = element;
		
		// JMenuItem item = new JMenuItem("Copy to...");
		
		add(createModifyMenuItem(element));
		add(createRemoveMenuItem(element));
		add(createCollapseCheckBox(element));
		add(new JSeparator());
		add(createCopyToPopup());
	}
	
}
