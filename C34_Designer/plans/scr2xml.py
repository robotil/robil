#!/usr/bin/python

import sys
import re

file = sys.argv[1]

def ReadFile(fname):
	return ''.join([line for line in open(fname,'r').readlines() if len(line.lstrip())>0 and line.lstrip()[0]!='#'])

text = ReadFile(file)

def loadIncludes(text, includedText):
	lines = [x.strip() for x in [y for y in text.split('\n') if len(y.lstrip())>0]if x.find('INCLUDE=')==0]
	for l in lines:
		k,v = l.split('=')
		includedText.append( ReadFile(v.strip()) )

includedText = []
loadIncludes(text, includedText)
if len(includedText)>0:
	text = text+'\n\nINCLUDED_FILES\n\n'+'\n\tFILE\n'.join(includedText)

#print text

import inspect
def __LINE__():
    try:
        raise Exception
    except:
        return sys.exc_info()[2].tb_frame.f_back.f_lineno
def __FILE__():
    return inspect.currentframe().f_code.co_filename
    
def onError_VariableNameDoesNotFound(w, repl, fromsrc):
	print "ERROR: ",onError_VariableNameDoesNotFound,w,'in',repl,'from line:',fromsrc
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
			fname =  functionName(node[0])
			if fname!='root' or 'root' not in functions:
				functions[fname]=node
	if 'root' not in functions:
		for f in functions:
			if f[0].find('root')==0:
				functions['root'] = f
		
func={}
splitFunctions(text, func)
#print 'keys = ', func.keys()

def splitDecoratorsAl(text, decorators):
	lines = [line.strip()[4:] for line in [line for line in text.split('\n') if len(line.lstrip())>0]if line.find('dec ')==0]
	for l in lines:
		k,v = l.split('=')
		decorators[k.strip()]=v.strip()

decorators={}
splitDecoratorsAl(text, decorators)

def isArray(t): return len(t)>2 and t[0]=='[' and t[-1]==']'

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

def compileXml( TAB, text, func, repl, defaultName):
	def getTab(t):
		res = ""
		for x in text:
			#print "["+x+"]"
			if x==t or x==' ':
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
					else:
						onError_VariableIsNotArray(ww);
						break
				else:
					t = t.replace(k,repl[w])
			else:
				onError_VariableNameDoesNotFound(w, repl,__LINE__());
				break
		return t
		
	def parseValues(dic, args, fname):
		key=fname+'.'
		val=''
		s=0;
		m=True
		for l,i in zip(args,xrange(len(args))):
			theLast = i==len(args)-1
			if theLast:
				if m: 
					if l!=' ': key=key+l
				else: 
					val=val+l
				dic[key.strip()]=val.strip()
				continue
			if s==0:
				if l=='=':
					m = not m
					continue
				if l == ',':
					m = not m
					dic[key.strip()]=val.strip()
					key=fname+'.'; val=''
					continue
				if l in '([{':
					s=s+1
				if l in '}])':
					s=s-1
				if m:
					if l!=' ': key=key+l
				else:
					val=val+l
			else:
				if l in '([{':
					s=s+1
				if l in '}])':
					s=s-1
				val=val+l
		#print dic
		
	def getAttribs(t):
		att = re.findall(r'attr\[[^]]+\]',t)
		t = re.sub(r'attr\[[^]]+\]','',t).strip()
		if len(att)>0: att = att[0][6:-1]
		else: att = ''
		return t, att
	
	def fparse(t, tab):
		global funcNameCounter
		ret = ''
		if len(t)<1: return ret
		t, attr = getAttribs(t)
		t = replasment(t)
		fname,args,decors = getPrefix(t)
		for d in decors:
			if d in decorators: d=decorators[d]
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
			elif fname.upper().find('ELSE')==0 :
				repl[args+'#']=0
				print tab+"<!-- "+'ELSE '+' -->'
				return tab+"<!-- "+'ENDELSE'+' -->'+ret
			else:
				typ = ''; tt = ''
				if fname.find('??')==0 : typ = 'swi'; tt='??'
				elif fname.find('?')==0 : typ = 'sel'; tt='?'
				elif fname.find('||')==0 : typ = 'par'; tt='||'
				if len(tt)>0: fname = fname[len(tt):].strip()
				if typ=='': typ = 'seq'; tt=''
				if len(fname)==0:
					if len(defaultName)==0:
						fname="@"+str(funcNameCounter)
						funcNameCounter+=1
					else:
						fname=defaultName
				if fname == '': fname = defaultName
				print tab+"<"+typ+' name="'+fname+'" '+ attr +'>'
				return tab+"</"+typ+">"+ret
		else:
			if fname.upper().find('NEXT')==0:
				if args not in repl:
					onError_VariableNameDoesNotFound(args, repl,__LINE__());
					return ret
				if not isArray(repl[args]):
					onError_VariableIsNotArray(args+'='+repl[args]);
					return ret
				if args+'#' not in repl: repl[args+'#']=0
				c = repl[args+'#']
				c=(c+1)%len(repl[args][1:-1].split(','))
				repl[args+'#'] = c
				print tab[:-1]+'<!-- next '+args+' -->'
				return ret
			elif fname in func:
				declar, body, strpos, endpos = func[fname]
				print tab+"<!-- function: "+ t + " -->"
				nrepl = repl.copy()
				parseValues(nrepl,args,fname)
				compileXml( tab, body, func, nrepl, fname)
				return tab+"<!-- function end: "+ fname+" -->"+ret
			else:
				if len(fname)==0:
					if len(defaultName)==0:
						fname="@"+str(funcNameCounter)
						funcNameCounter+=1
					else:
						fname=defaultName
				if len(args)>0:
					print tab+'<tsk name="'+fname+'('+args+')'+'" '+ attr +' />'
				else:
					print tab+'<tsk name="'+fname+'" '+ attr +'/>'
				return ret
		
	tab = TAB+getTab('\t')
	#print "tab=["+tab+"]"
	while len(text.strip())>0:
		#print "text= ",text
		line = getLine(text,0);
		linelen = len(line)
		line = line.strip()
		wattr,attr = getAttribs(line)
		if len(line)<1:
			print tab+"<!-- empty line -->"
		elif line=="}":
			pass
		elif wattr[-1] != '{':
			tg = fparse(line, tab)
			if len(tg)>0: print tg
		else:
			tg = fparse(line, tab)
			end = findEnd(text,text.find('{'),'{','}')
			ttt = text[linelen+1:end]
			linelen = end+1
			fname,args,decors = getPrefix(line)
			if fname.upper().find('FOR')==0:
				if args not in repl:
					onError_VariableNameDoesNotFound(args, repl,__LINE__())
					continue
				if not isArray(repl[args]):
					onError_VariableIsNotArray(args+'='+repl[args])
					continue
				if args in repl:
					vals = [x.strip() for x in repl[args][1:-1].split(',')]
					for xi,x in enumerate(vals):
						repl[args+'#']=xi
						compileXml( TAB, ttt, func, repl, "")
						if xi!=len(vals)-1: print tab+'<!-- NEXT '+args+' -->'
			elif fname.upper().find('IF')==0:
				args_k,args_v = args.split('=')
				args_k = args_k.strip()
				args_v = args_v.strip()
				isnot = False
				if args_k.find('!')==0:
					args_k=args_k[1:]
					isnot = True
				if len(args_k.rstrip())>0 and args_k.rstrip()[-1]=='!':
					args_k=args_k[:-1]
					isnot = True
				if args_k in repl:
					vv = ''
					if args_k[-1]!='#': vv = repl[args_k]
					else:
						if args_k[:-1] not in repl:
							onError_VariableNameDoesNotFound(args, repl,__LINE__())
							continue
						val = repl[args_k[:-1]]
						if not isArray(val):
							onError_VariableIsNotArray(args+'='+repl[args])
							continue
						val = val[1:-1]
						vv = val.split(',')[repl[args_k]].strip()
					#print "compare: ",'['+vv+']', isnot, '['+args_v+']',(vv==args_v), (isnot==False and vv==args_v), (isnot==True and vv!=args_v)
					if isnot==False and vv==args_v:
						compileXml( TAB, ttt, func, repl, "")
					if isnot==True and vv!=args_v:
						compileXml( TAB, ttt, func, repl, "")
			else:
				compileXml( TAB, ttt, func, repl, "")
			print tg
		text = text[linelen+1:]

if 'root' not in func:
	print "root element not found"
	exit(1)
		
print '<plan>'
compileXml( '', "root", func , {}, "")
print '</plan>'
