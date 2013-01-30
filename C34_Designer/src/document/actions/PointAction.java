package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;

public class PointAction extends AbstractDesignerAction implements
		ActionListener {

	public PointAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		Document doc = getActiveTab().doc;
		doc.toolSelectionClean();
		this.designer.toolbar.setTipText(Toolbar.TIP_move);
	}

}
