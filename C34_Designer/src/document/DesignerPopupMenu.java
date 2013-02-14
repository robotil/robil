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
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;

import document.BTDesigner.DesignerTab;
import document.actions.CopyToAction;

import elements.Arrow;
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
			
			item.addActionListener(new CopyToAction(_designer, this._element, tab.doc));
			
			menu.add(item);
		}
		
		return menu;
	}
	
	private JCheckBoxMenuItem createCollapseCheckBox() {
		final JCheckBoxMenuItem menu = new JCheckBoxMenuItem("Collapsed");
		menu.setState(_element.getProperty().collapsed);
		// menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/remove.png")));
		menu.addChangeListener(new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent arg) {
				_element.getProperty().collapsed = menu.getState();
			}
		});
		
		return menu;
	}
	
	private JMenuItem createRemoveMenuItem() {
		JMenuItem menu = new JMenuItem("Remove");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/remove.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				_document.remove(_element);
			}
		});
		return menu;
	}
	
	private JMenuItem createRemoveSubtreeMenuItem() {
		JMenuItem menu = new JMenuItem("Remove subtree");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/remove.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				_document.removeSubTree(_element);
			}
		});
		return menu;
	}
	
	private JMenuItem createModifyMenuItem() {
		JMenuItem menu = new JMenuItem("Modify");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/modify.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				_element.modify();
			}
		});
		return menu;
	}
	
	private JMenuItem createCopyMenuItem() {
		JMenuItem menu = new JMenuItem("Copy");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/copy.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				_document.copyTree(_element);
			}
		});
		return menu;
	}
	
	private JMenuItem createTaskCreateMenuItem() {
		JMenuItem menu = new JMenuItem("Create task...");
		menu.setIcon(new ImageIcon(getClass().getClassLoader().getResource("icons/add_icon.png")));
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg) {
				_document.createTask();
			}
		});
		return menu;
	}
	
	private JMenuItem createJointMenuItem() {
		JMenuItem menu = new JMenuItem("Create joint");
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg) {
				_document.createJoint(_element);
			}
		});
		return menu;
	}
	
	private JMenuItem createDecoratorMenuItem() {
		JMenuItem menu = new JMenuItem("Create decorator...");
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg) {
				_document.createDecorator(_element);
			}
		});
		return menu;
	}
	
	private JMenuItem createReconnectMenuItem() {
		JMenuItem menu = new JMenuItem("Reconnect");
		menu.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg) {
				DesignerPopupMenu.this._document.creator = new Arrow.Reconector((Arrow) DesignerPopupMenu.this._element);
			}
		});
		return menu;
	}
	
	public void createArrowMenu() {
		add(createJointMenuItem());
		add(createDecoratorMenuItem());
		add(createReconnectMenuItem());
		add(new JSeparator());
		add(createRemoveMenuItem());
	}
	
	public void createDecoratorMenu() {
		add(createModifyMenuItem());
		add(createRemoveMenuItem());
	}
	
	public void createTaskMenu() {
		add(createModifyMenuItem());
		add(createCopyMenuItem());
		add(createCopyToPopup());
		if (!this._element.isTaskType())
			add(createCollapseCheckBox());
		
		add(new JSeparator());
		add(new JSeparator());
		
		add(createRemoveMenuItem());
		if (!this._element.isTaskType())
			add(createRemoveSubtreeMenuItem());
	}
	
	/**
	 * Creates a popup menu for selected element
	 * @param designer BTDesigner object
	 * @param document Target document
	 * @param element Selected element
	 */
	public DesignerPopupMenu(BTDesigner designer, Document document, GElement element) {
		_designer = designer;
		_document = document;
		_element = element;

		if (element.isArrow())
			createArrowMenu();
		
		if (element.isTask()) 
			createTaskMenu();
		
		if (element.isDecorator())
			createDecoratorMenu();
		
		initPopupMenu();
	}
	
	/**
	 * Creates popup menu when the document is clicked, but none elements selected
	 * @param designer BTDesigner object
	 * @param document Target document
	 */
	public DesignerPopupMenu(BTDesigner designer, Document document) {
		_designer = designer;
		_document = document;
		
		add(createTaskCreateMenuItem());
		
		initPopupMenu();
	}
	
	private void initPopupMenu() {
		this.addPopupMenuListener(new PopupMenuListener() {
			@Override
			public void popupMenuWillBecomeVisible(PopupMenuEvent arg0) {}
			@Override
			public void popupMenuCanceled(PopupMenuEvent arg0) {}
			@Override
			public void popupMenuWillBecomeInvisible(PopupMenuEvent arg0) {
				_document.repaint();
			}
		});
	}
	
}
