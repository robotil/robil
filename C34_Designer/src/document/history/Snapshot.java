package document.history;

import java.util.ArrayList;
import java.util.Date;

import document.Document;
import elements.GElement;

class Snapshot {
	
	private Date _date;
	private Document _document;
	private int _sequenceNumber;
	ArrayList<GElement> _elements;
	ArrayList<GElement> _arrows;
	
	private Snapshot(Document document, ArrayList<GElement> elements, ArrayList<GElement> arrows, int sequenceNumber) {
		this._date 		= new Date();
		this._document 	= document;
		this._elements 	= elements;
		this._arrows 	= arrows;
		this._sequenceNumber = sequenceNumber;
	}
	
	public void activate() {
		this._document.elements = this._elements;
		this._document.arrays  	= this._arrows;

		for (GElement element : this._elements) 
			element.setView(this._document.view);
		
		for (GElement element : this._arrows) 
			element.setView(this._document.view);

		this._document.repaint();
	}
	
	public static Snapshot create(Document document, int sequenceNumber) {
		ArrayList<GElement> elements = new ArrayList<GElement>();
		ArrayList<GElement> arrows   = new ArrayList<GElement>();
		document.cloneElements(elements, arrows);
		
		// assert elements.size() == document.elements.size() : elements;
		// assert arrows.size() == document.arrays.size() : arrows;
		
		return new Snapshot(document, elements, arrows, sequenceNumber);
	}
	
	public Date getDate() {
		return _date;
	}
	
	@Override
	public String toString() {
		return Integer.toString(this._sequenceNumber) + "X" + _date.getTime();
	}
}
