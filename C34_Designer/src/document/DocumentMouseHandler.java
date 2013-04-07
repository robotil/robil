package document;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

import document.Document.TreeChangeType;
import document.ui.DesignerPopupMenu;
import elements.Arrow;
import elements.GElement;
import elements.Vec;
import elements.View;

public class DocumentMouseHandler extends MouseAdapter {

	private Document _document;
	
	public DocumentMouseHandler(Document document) {
		this._document = document;
	}
	
	@Override
	public void mouseDragged(MouseEvent e) {
		if (this._document.mousePressed == null)
			return;
		if (this._document.selectedElement == null) {
			// Vec d = new Vec(e.getPoint()).sub(new
			// Vec(mousePressed)).scale(1/view.zoom);
			this._document.view.loc.setOffset(new Vec(e.getPoint()).sub(new Vec(this._document.mousePressed)));
			this._document.mousePressed = e.getPoint();
		} else {
			Vec d = new Vec(e.getPoint()).sub(new Vec(this._document.mousePressed)).scale(1 / this._document.view.zoom);
			this._document.selectedElement.getProperty().location.setOffset(d);
			for (GElement el : this._document.searchAllSubelements(this._document.selectedElement)) {
				el.getProperty().location.setOffset(d);
			}
			this._document.mousePressed = e.getPoint();
		}
		this._document.repaint();
		super.mouseDragged(e);
	}

	@Override
	public void mousePressed(MouseEvent ev) {
		
		Vec mousePosition = new Vec(ev.getPoint()).sub(this._document.view.loc).scale(1 / this._document.view.zoom);
		this._document.setMousePosition(mousePosition);
		
		this._document.mousePressed = ev.getPoint();
		for (GElement el : this._document.getReversed(this._document.elements)) {
			GElement e = el.underMouse(ev.getPoint());
			if (e != null && e.isVisiable) {
				this._document.selectedElement = e;
				
				if (ev.getButton() == MouseEvent.BUTTON1)
					this._document.selectedElement.getProperty().leftClicked = true;
				else if (ev.getButton() == MouseEvent.BUTTON3)
					this._document.selectedElement.getProperty().rightClicked = true;
				
				this._document.repaint();
				break;
			}
		}
		if (this._document.selectedElement == null)
			for (GElement el : this._document.arrays) {
				GElement e = el.underMouse(ev.getPoint());
				if (e != null && e.isVisiable) {
					this._document.selectedElement = e;
					
					if (ev.getButton() == MouseEvent.BUTTON1)
						this._document.selectedElement.getProperty().leftClicked = true;
					else if (ev.getButton() == MouseEvent.BUTTON3)
						this._document.selectedElement.getProperty().rightClicked = true;
					
					this._document.repaint();
					break;
				}
			}
		
		super.mousePressed(ev);
	}
	
	@Override
	public void mouseReleased(MouseEvent e) {
		this._document.mousePressed = null;
		
		if (e.getButton() == MouseEvent.BUTTON1) {
		
			if (this._document.creator != null) {
				if (this._document.selectedElement != null)
					this._document.creator.add(this._document.selectedElement);
				boolean checkCreator = this._document.creator != null
						&& this._document.creator.ready()
						&& ((this._document.selectedElement == null && this._document.creator
								.createOnEmptyPlace()) || (this._document.selectedElement != null && !this._document.creator
								.createOnEmptyPlace()));
				if (checkCreator) {
					Vec p = new Vec(e.getPoint()).sub(this._document.view.loc)
							.scale(1 / this._document.view.zoom);
					GElement el = this._document.creator.newInstance();
					if (el instanceof Arrow) {
						Arrow a = (Arrow) el;
						this._document.onBeforeTreeChange(TreeChangeType.ArrowModify, el);
						if (this._document.getArrow(a.getSource(), a.getTarget()).size() > 0
								|| this._document.getArrow(a.getTarget(), a.getSource())
										.size() > 0)
							el = null;
						this._document.onTreeChange(TreeChangeType.ArrowModify, el);
					}
					if (el != null) {
						this._document.onBeforeTreeChange(TreeChangeType.ArrowModify, el);
						el.setView(this._document.view);
						if (el instanceof View.ChangesListener)
							((View.ChangesListener) el).onViewChange();
						el.getProperty().setCenter(p);
						this._document.add(el);
						el.modify();
						this._document.repaint();
						this._document.onTreeChange(TreeChangeType.ArrowModify, el);
					}
					if (this._document.cleanToolSelectionAfterUse)
						this._document.toolSelectionClean();
					else if (this._document.creator instanceof Arrow.Reconector) {
						((Arrow.Reconector) this._document.creator)
								.getInstance().getProperty().leftClicked = false;
						this._document.toolSelectionClean();
					}
				}
			}
			if (this._document.removeElement
					&& this._document.selectedElement != null) {
				this._document.remove(this._document.selectedElement);
				if (this._document.cleanToolSelectionAfterUse)
					this._document.toolSelectionClean();
			}
			if (this._document.removeSubElements
					&& this._document.selectedElement != null) {
				this._document.removeSubTree(this._document.selectedElement);
				// if(cleanToolSelectionAfterUse)
				this._document.toolSelectionClean();
			}
			if (this._document.copyElement
					&& this._document.selectedElement != null) {
				this._document.copyTree(this._document.selectedElement);
				// if(cleanToolSelectionAfterUse)
				this._document.toolSelectionClean();
			}
			if (this._document.reconectArrow
					&& this._document.selectedElement != null
					&& this._document.selectedElement instanceof Arrow) {
				this._document.creator = new Arrow.Reconector(
						(Arrow) this._document.selectedElement);
				this._document.mainWindow.toolbar
						.setTipText(this._document.creator.toolTip());
			}
			if (this._document.modifier != null
					&& this._document.selectedElement != null) {
				this._document.modifier.set(this._document.selectedElement);
				if (this._document.cleanToolSelectionAfterUse)
					this._document.toolSelectionClean();
			}
			if (this._document.selectedElement == null)
				return;

			this._document.selectedElement.getProperty().leftClicked = false;
			this._document.selectedElement = null;

			if (this._document.creator != null
					&& this._document.creator instanceof Arrow.Reconector) {
				((Arrow.Reconector) this._document.creator).getInstance()
						.getProperty().leftClicked = true;
			}

			this._document.repaint();
			super.mouseReleased(e);
			
		} else if (e.getButton() == MouseEvent.BUTTON3) {
			// Right click
			this._document.toolSelectionClean();
			if (this._document.selectedElement != null) {
				DesignerPopupMenu popup = new DesignerPopupMenu(this._document.mainWindow, this._document, this._document.selectedElement);
				popup.show(e.getComponent(), e.getX(), e.getY());
				this._document.selectedElement.getProperty().rightClicked = false;
				this._document.selectedElement = null;
			} else {
				DesignerPopupMenu popup = new DesignerPopupMenu(this._document.mainWindow, this._document);
				popup.show(e.getComponent(), e.getX(), e.getY());
			}
			
		}
		
		
	}

	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
		int notches = e.getWheelRotation();
		if (notches < 0 && this._document.view.zoom < 0.3)
			return;
		if (notches > 0 && this._document.view.zoom > 5)
			return;
		double old_zoom = this._document.view.zoom;
		double new_zoom = this._document.view.zoom = this._document.view.zoom + (notches * 0.1);
		Vec m = new Vec(e.getPoint());
		Vec d = this._document.view.loc.sub(m).scale(1.0 / old_zoom)
				.scale(new_zoom);
		this._document.view.loc = d.add(m);
		this._document.repaint();
		super.mouseWheelMoved(e);
	}

}