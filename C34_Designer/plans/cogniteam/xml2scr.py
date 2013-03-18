#!/usr/bin/python

import sys
import re

file = sys.argv[1]

text = open(file,'r').read()

if 'remove_attr' in sys.argv:
	print re.sub(r'attr\[[^]]*\]',r'',text)
	exit(0)

text = text \
	.replace('<plan>','def root{') \
	.replace('</plan>','}');

def makeIndent(text):
	lines = text.split('\n')
	tab=''
	tabinc='\t'
	nlines = []
	for line in lines:
		line = orig = line.strip()
		line = re.sub(r'attr\[.*\]',r'',line).strip()
		if len(line)<1:
			nlines.append("")
			continue
		if line[-1]=='{':
			nlines.append(tab+orig)
			tab+=tabinc
		elif line[-1]=='}':
			tab = tab[:-len(tabinc)]
			nlines.append(tab+orig)
		else:
			nlines.append(tab+orig)
			
	return '\n'.join(nlines)

def replace(text):

	text = re.sub(r'(\n?)\s*</dec>\s*(\n)',r'\1',text)
	text = re.sub('</.+>','}',text)
	text = re.sub(r'<tsk name="([^"]*)"(.*)/>',r'\1 attr[\2]',text)
	text = re.sub(r'<seq name="([^"]*)"(.*)>',r'\1{ attr[\2]',text)
	text = re.sub(r'<sel name="([^"]*)"(.*)>',r'?\1{ attr[\2]',text)
	text = re.sub(r'<par name="([^"]*)"(.*)>',r'||\1{ attr[\2]',text)
	text = re.sub(r'<swi name="([^"]*)"(.*)>',r'??\1{ attr[\2]',text)
	text = re.sub(r'<dec name="([^"]*)".*\n\s*',r'<\1>',text)

	text = makeIndent(text)
	
	return text


def extract_functions(text):
	lines = text.split('\n')

	def getId(text): return re.findall(r'id="(.*)"',text)

	ids = getId(text)
	#print ids
	ids_count={}
	for id in ids:
		if id in ids_count: ids_count[id]+=1
		else: ids_count[id]=1

	#print ids_count

	def countLines(lines, ind):
		str = lines[ind].strip()
		if str.find('<tsk')==0: return 1
		tag = str[1:4]
		c=0
		for i in xrange(ind,len(lines)):
			if lines[i].strip().find('<'+tag)==0: c+=1
			if lines[i].strip().find('</'+tag)==0: c-=1
			if c<0: return None
			if c==0: return i-ind
		return None

	extracted_func={}
	for indx, line in enumerate(lines):
		id = getId(line)
		#print "line -> ",id
		if len(id)>0:
			id = id[0]
			if ids_count[id]>1:
				count = countLines(lines,indx)
				if count!=None and count>1:
					code = lines[indx:indx+count+1]
					strblocks = [i for i in xrange(indx,len(lines)) if lines[i]==line][::-1]
					fname = re.findall(r'name="([^"]*)"',line)[0]
					for b in strblocks:
						lines = lines[:b]+['func_'+fname]+lines[b+count+1:]
					extracted_func[fname] = code
					break
		
	return '\n'.join(lines), extracted_func

def createFunctions(text):
	defs = []
	for f in ef:
		defs.append( 'def func_'+str(f)+"{" )
		defs+=ef[f]
		defs.append( '}' )
	 
	return text + '\n'*2+'\n'.join(defs)

text, ef = extract_functions(text)
text = createFunctions(text)
text = replace(text)



print text

#print re.findall(r'id="(.*)"',text)



