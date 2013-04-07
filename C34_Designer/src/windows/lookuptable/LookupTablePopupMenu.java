package windows.lookuptable;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JTable;

import document.LookupTable;
import document.ui.Utilities;

public class LookupTablePopupMenu extends JPopupMenu {

	private static final long serialVersionUID = 1L;
	
	public LookupTablePopupMenu(final LookupTable lookupTable, final int rowIndex, final JTable jTable) {
		add(new JMenuItem("Delete", Utilities.loadIcon("remove.png")) {
			private static final long serialVersionUID = 305796429836013352L;
			{
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent arg) {
						lookupTable.remove(rowIndex);
						jTable.repaint();
					}
				});
			}
		});
	}
}
