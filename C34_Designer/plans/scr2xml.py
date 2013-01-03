#!/usr/bin/python

import sys

file = sys.argv[1]

text = open(file,'r').read()

#print text

def onError_VariableNameDoesNotFound(w):
	print "ERROR: ",onError_VariableNameDoesNotFound,w
	pass
def onError_VariableIsNotArray(w):
	print "ERROR: ",onError_VariableIsNotArray,w
	pass

def checkStructure(t, odel,cdel):
	line=1
	pos=0
	c=0
	for x in t:
		pos+=1
		if x=='\t': pos+=3
		if x=='\n':
			line+=1
			pos=1
		if x==odel: c+=1
		if x==cdel: c-=1
		if c<0: return (False,1,line,pos)
	if c>0: return (False,2,0,0)
	return (True,0,0,0)
	
def findEnd(t, s, odel,cdel):
	t = t[s:]
	pos=0
	c=0
	for x in t:
		pos+=1
		if x==odel: c+=1
		if x==cdel: c-=1
		if c<0: return -1
		if c==0: return s+pos+1;
	return -1
	
def getLine(t, s):
	t = t[s:]
	e = t.find('\n')
	if e<0: return t;
	return t[:e]

r = checkStructure(text, '{', '}')
if not r[0]: print '{}', r
r = checkStructure(text, '(', ')')
if not r[0]: print '()', r
r = checkStructure(text, '[', ']')
if not r[0]: print '[]', r
r = checkStructure(text, '<', '>')
if not r[0]: print '<>', r

def splitLines(t,s):
	lines = []
	c = t.find('\n',s);
	while c >=0:
		lines.append(t[s:c+1])
		s=c+1
		c=t.find('\n',s);
	if s<len(t)-1 : lines.append(t[s:len(t)-1])
	return lines
	
def functionName(f):
	f = f.strip()
	s = f.find('(')
	if s>=0: return f[:s]
	return f
def functionArgs(f):
	f = f.strip()
	s = f.find('(')
	if s>=0: return f[s+1:-1]
	return ""

def splitFunctions(text, functions):
	def searchTag(tag, str):
		tag+=' '
		root_str = text.find(tag, str)
		if root_str>=0:
			line = getLine(text,root_str)
			p = line.find('{')
			if p>=0 :
				root_end = findEnd(text,root_str+p,'{','}')
				return line.split(tag)[1][:-1].strip(), ''.join(splitLines(text[root_str:root_end],0)[1:-1]), root_str, root_end
	node = searchTag('def', 0)
	start = 0
	while node!=None:
		node = searchTag('def', start)
		if node!=None:
			start = node[3]
			functions[functionName(node[0])]=node
	for f in functions:
		if f[0].find('root')==0:
			functions['root'] = f
		
func={}
splitFunctions(text, func)
#print 'keys = ', func.keys()

funcNameCounter=1
def getPrefix(txt):
	rev = txt.strip()[::-1]
	arguments=""
	funcname=""
	decors=[]
	if rev[0]=='{': rev = rev[1:].strip()
	if len(rev)<1: return funcname, arguments, decors
	if rev[0]==')':
		e = findEnd(rev,0,')','(')-1
		arguments = rev[:e][1:-1][::-1].strip()
		rev = rev[e:]
	if ' ' in rev or '>' in rev:
		si = rev.find(' ')
		if si<0: si = len(rev)
		ti = rev.find('>')
		if ti<0: ti = len(rev)
		mini = min(si,ti)
		funcname = rev[:mini][::-1]
	else: funcname = rev[::-1]
	rev = rev[len(funcname):].strip()[::-1]
	if '<' in rev:
		if ' ' in rev and rev.find(' ')<rev.find('<'): rev = rev[rev.find(' '):].strip()
		rev = [[decors.append(a.strip()) for a in x.split('>') if len(a.strip())>0] for x in [y for y in rev.split('<')] ]

	return funcname, arguments, decors

def compileXml( TAB, text, func, repl):
	def getTab(t):
		res = ""
		for x in text:
			#print "["+x+"]"
			if x==t:
				#print x
				res+=x
			else: return res
		return res
	def replasment(t):
		while t.find('$[')>=0:
			s = t.find('$[')
			e = findEnd(t,s+1,'[',']')-1
			w = t[s+2:e-1].strip()
			if len(w)<1 : break
			ww=w
			iscounter=False
			if w[-1]=='#':
				w = w[:-1]
				iscounter=True
			if w in repl:
				k='$['+ww+']'
				if iscounter:
					v = repl[w]
					if v[0] in '[{(' and v[-1] in ')}]':
						if (w+'#') not in repl: repl[w+'#']=0
						c = repl[w+'#']
						t = t.replace(k,v[1:-1].split(',')[c])
						c=(c+1)%len(v[1:-1].split(','))
						repl[w+'#'] = c
					else:
						onError_VariableIsNotArray(ww);
						break
				else:
					t = t.replace(k,repl[w])
			else:
				onError_VariableNameDoesNotFound(ww);
				break
		return t
		
	def parseValues(dic, args, fname):
		if '=' not in args: return
		te = args.split('=')
		if len(te)==2: dic[te[0]]=te[1]
		k=fname+'.'+te[0]
		for x in xrange(1,len(te)-1):
			v = te[x][::-1]
			kk = v[v.find(','):][::-1]
			kk = fname+'.'+kk
			v = v[:v.find(',')][::-1]
			dic[k]=v
			k=kk
		dic[k]=te[-1]
	
	def fparse(t, tab):
		global funcNameCounter
		ret = ''
		if len(t)<1: return ret
		t = replasment(t)
		fname,args,decors = getPrefix(t)
		for d in decors:
			print tab+'<dec name="'+d+'" >'
			if len(ret)==0: ret = tab+'</dec>'
			else: ret = tab+'</dec>'+'\n'+ret
		
		if t[-1]=='{':
			if fname.upper().find('FOR')==0 :
				repl[args+'#']=0
				print tab+"<!-- "+'FOR '+' '+args+' -->'
				return tab+"<!-- "+'DONE'+' '+args+' -->'+ret
			elif fname.upper().find('IF')==0 :
				repl[args+'#']=0
				print tab+"<!-- "+'IF '+' '+args+' -->'
				return tab+"<!-- "+'ENDIF'+' '+args+' -->'+ret
			else:
				typ = ''
				if fname.find('??')==0 : typ = 'swi'
				elif fname.find('?')==0 : typ = 'sel'
				elif fname.find('||')==0 : typ = 'par'
				if len(typ)>0: fname = fname[len(typ)-1:].strip()
				if typ=='': typ = 'seq'
				if len(fname)==0:
					fname="@"+str(funcNameCounter)
					funcNameCounter+=1
				print tab+"<"+typ+' name="'+fname+'">'
				return tab+"</"+typ+">"+ret
		else:
			#print ">>> ",fname
			if fname in func:
				declar, body, strpos, endpos = func[fname]
				print tab+"<!-- function: "+ t + " -->"
				nrepl = repl.copy()
				parseValues(nrepl,args,fname)
				compileXml( tab, body, func, nrepl)
				return tab+"<!-- function end: "+ fname+"-->"+ret
			else:
				if len(fname)==0:
					fname="@"+str(funcNameCounter)
					funcNameCounter+=1
				if len(args)>0:
					print tab+'<tsk name="'+fname+'('+args+')'+'" />'
				else:
					print tab+'<tsk name="'+fname+'" />'
				return ret
		
	tab = TAB+getTab('\t')
	#print "tab=["+tab+"]"
	while len(text.strip())>0:
		#print "text= ",text
		line = getLine(text,0);
		linelen = len(line)
		line = line.strip()
		if len(line)<1:
			print tab+"<!-- empty line -->"
		elif line=="}":
			pass
		elif line[-1] != '{':
			tg = fparse(line, tab)
			if len(tg)>0: print tg
		else:
			tg = fparse(line, tab)
			end = findEnd(text,text.find('{'),'{','}')
			ttt = text[linelen+1:end]
			linelen = end+1
			fname,args,decors = getPrefix(line)
			if fname.upper().find('FOR')==0:
				if args in repl:
					for x in repl[args].split(','):
						compileXml( TAB, ttt, func, repl)
			elif fname.upper().find('IF')==0:
				args_k,args_v = args.split('=')
				if args_k in repl and repl[args_k]==args_v:
					compileXml( TAB, ttt, func, repl)
			else:
				compileXml( TAB, ttt, func, repl)
			print tg
		text = text[linelen+1:]

if 'root' not in func:
	print "root element not found"
	exit(1)
		
print '<plan>'
compileXml( '', "root", func , {'A':'[10,11]'})
print '</plan>'
