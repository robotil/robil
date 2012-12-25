package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;

public class ArrowReconnectAction extends AbstractDesignerAction implements ActionListener {
	
	public ArrowReconnectAction(BTDesigner designer) {
		super(designer);
	}
	
	public void actionPerformed(ActionEvent a) {
		Document document = getActiveTab().doc;
		document.toolSelectionClean();
		document.reconectArrow = true;
		designer.toolbar.setTipText(Toolbar.TIP_reconnect);
	}	

}
