/*
 * MapFileReader.hpp
 *
 *  Created on: Mar 3, 2013
 *      Author: dan
 */

#ifndef MAPFILEREADER_HPP_
#define MAPFILEREADER_HPP_


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

namespace map_file_reader{

using namespace std;

struct CanNotOpenFile{};
struct CanNotFindDimentions{};
struct Dim{ int w; int h; bool reversed; };

static Dim getDim(const vector<string>& lines){
	if(lines.size()<2) throw CanNotFindDimentions();
	Dim d={0,0, false};
	stringstream ww(lines[0]);
	while(ww>>d.w);
	stringstream h1(lines[1]);
	stringstream h2(lines[lines.size()-1]);
	h1>>d.h;
	if(d.h==0){
		h2>>d.h;
		d.reversed=false;
	}else{
		d.reversed=true;
	}
	d.w+=1;
	d.h+=1;
	return d;
}

static vector<char> readMap(string fname, size_t& w, size_t& h, bool verb=false){
	ifstream f(fname.c_str());
	stringstream sl;
	string line;
	vector<string> lines;
	//int w,h;
	vector<char> map;
	while( getline(f, line) ){
		lines.push_back(line);
	}
	Dim dim = getDim(lines);
	w = dim.w; h=dim.h;
	if(verb) cout<<"Dim: "<<w<<"x"<<h<<(dim.reversed?" reversed":"")<<endl;

	size_t s, e, inc;
	if(dim.reversed){
		s=1+dim.h-1; e=0; inc=-1;
	}else{
		s=1; e=s+dim.h; inc=1;
	}

	for(size_t i=s; i!=e; i+=inc){
		stringstream sl(lines[i]);

		int n;
		if(!(sl>>n)) break;
		while(true){
			char c;
			if(!(sl>>c)) break;
			if(c=='-' || c=='.' || c=='B'){
				if(c=='.') n=0;
				if(c=='-') n=2;
				if(c=='B') n=1;
			}
			//cout<<n;
			if(verb) cout<<c;
			map.push_back((char)n);
		}
		if(verb) cout<<endl;
	}

	return map;
}

}

#endif /* MAPFILEREADER_HPP_ */
