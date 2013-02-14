package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;
import elements.GElement;

public class CopyToAction extends AbstractDesignerAction implements
		ActionListener {

	private GElement _element;
	private Document _targetDocument;

	public CopyToAction(BTDesigner designer, GElement element, Document targetDocument) {
		super(designer);
		this._element = element;
		this._targetDocument = targetDocument;
	}

	@Override
	public void actionPerformed(ActionEvent a) {
//		Document document = getActiveTab().doc;
//		document.toolSelectionClean();
//		document.copyElement = true;
//		
		this._targetDocument.copyTree(this._element, getActiveTab().doc);
		this.designer.setActiveTab(this._targetDocument);
		this.designer.toolbar.setTipText(Toolbar.TIP_copy);
	}

}
