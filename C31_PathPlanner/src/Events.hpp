

#ifndef _EVENTS_C31_H_
#define _EVENTS_C31_H_

#include <string>
#include <boost/thread.hpp>
#include <vector>
#include <C31_PathPlanner/C31_Exception.h>

struct Event{
public:
	enum Type{
		TYPE_UKNOWN=0,
		TYPE_NOSOLUTIONFORPLAN=1,
		TYPE_ROBOTOUTOFMAP=2,
		TYPE_MAPISEMPTY=3,
		TYPE_FAILTEDTOCALLSERVICEC22=4
	} type;
	std::string description;
	Event(Type t, std::string d):type(t), description(d){}
	Event(const Event& e):type(e.type), description(e.description){}
	Event& operator=(const Event& e){ type=(e.type); description=(e.description); return *this;}
};

class Events{
private:
	boost::mutex mtx;
	std::vector<std::vector<Event>*> vec;
public:

	class Queue{
		Events& ev;
		std::vector<Event> vec;
	public:
		Queue(Events& ev):ev(ev){
			ev.add(&vec);
		}
		~Queue(){ev.remove(&vec);}

		bool empty(){
			return ev.empty(&vec);
		}
		Event pop(){
			return ev.pop(&vec);
		}
	};


	void add(std::vector<Event>* v){
		boost::mutex::scoped_lock l(mtx);
		vec.push_back(v);
	}
	void remove(std::vector<Event>* v){
		boost::mutex::scoped_lock l(mtx);
		vec.push_back(v);
	}
	void push(const Event& e){
		boost::mutex::scoped_lock l(mtx);
		for(size_t i=0;i<vec.size();i++) vec[i]->push_back(e);
	}

protected:
	bool empty(std::vector<Event>* vec){
		boost::mutex::scoped_lock l(mtx);
		return vec->empty();
	}
	Event pop(std::vector<Event>* vec){
		boost::mutex::scoped_lock l(mtx);
		if(empty(vec)) return Event(Event::TYPE_UKNOWN,"");
		Event e = vec->front();
		vec->erase(vec->begin());
		return e;
	}
};

inline Event cast(C31_PathPlanner::C31_Exception& e){
	switch(e.type){
	case C31_PathPlanner::C31_Exception::TYPE_FAILTEDTOCALLSERVICEC22: return Event(Event::TYPE_FAILTEDTOCALLSERVICEC22, e.description);
	case C31_PathPlanner::C31_Exception::TYPE_MAPISEMPTY: return Event(Event::TYPE_MAPISEMPTY, e.description);
	case C31_PathPlanner::C31_Exception::TYPE_NOSOLUTIONFORPLAN: return Event(Event::TYPE_NOSOLUTIONFORPLAN, e.description);
	case C31_PathPlanner::C31_Exception::TYPE_ROBOTOUTOFMAP: return Event(Event::TYPE_ROBOTOUTOFMAP, e.description);
	case C31_PathPlanner::C31_Exception::TYPE_UKNOWN: return Event(Event::TYPE_UKNOWN, e.description);
	}
	return Event(Event::TYPE_UKNOWN,"");
}


#endif
