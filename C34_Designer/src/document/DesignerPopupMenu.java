package document;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Action;
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
import elements.GElement;
import elements.Task;

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
	
	public DesignerPopupMenu(BTDesigner designer, Document document, GElement element) {
		_designer = designer;
		_document = document;
		_element = element;
		
		// JMenuItem item = new JMenuItem("Copy to...");
		
		if (!element.isArrow())
			add(createModifyMenuItem());
		
		if (element.isArrow()) {
			add(createJointMenuItem());
			add(createDecoratorMenuItem());
		}
		
		
		if (!element.isTaskType() && !element.isArrow())
			add(createCollapseCheckBox());
		
		add(new JSeparator());
		
		add(createRemoveMenuItem());
		
		if (!element.isArrow())
			add(createCopyMenuItem());
		
		if (!element.isArrow())
			add(createCopyToPopup());
		

		
		initPopupMenu();
	}
	
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
