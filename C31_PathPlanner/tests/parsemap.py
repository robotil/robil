#!/usr/bin/python

#print 'START'


def readlines():
	lines=[]
	import fileinput
	for line in fileinput.input():
		lines.append(line[:-1])
	return lines

#lines = [x[:-1] for x in open('t2' ,'r').readlines()]
lines = readlines()

def tostring(l): return "\n".join(l)
def maptostring(l): return "\n".join(["    "+v for v in [','.join(x) for x in l]])

def searchAltMap(lines):
	res=[]
	save=False
	for line in lines:
		if save: res.append(line)
		if not save and line.find('PRINT MAP map source')>=0:
			save = True
		if save and line=='':
			return res
	return res


def searchW(altmap):
	return int(altmap[0].strip().split(' ')[-1])+1

def searchTransfers(altmap):
	altmap = altmap[1:]
	r1=[]
	r2=[]
	def s(x): 
		if(x.find('-->')>=0):
			r1.append(x)
		else:
			r2.append(x)
	[s(x) for x in altmap]
	return r1, r2

def searchH(l):
	return int(l[0].strip().split(' ')[0])+1

def createTransfers(lines):
	m = {}
	maxval=-100000;
	for l in lines:
		k,v = l.strip().split(' --> ')
		m[v]=float(k)
		if maxval<m[v]: maxval = m[v]
	return m, maxval
def createAltMap(lines, transfers, maxval):
	m=[]
	for l in reversed(lines[:-1]):
		m.append(  [ str( transfers[x]/maxval*255 ) for x in l.strip().split(' ')[1:] if x.strip()!='' ]  )
	return m
		

altmap = searchAltMap(lines)
altmap_w = searchW(altmap)
altmap_transfers, rest = searchTransfers(altmap)
altmap_h = searchH(rest)
transfers, maxval = createTransfers(altmap_transfers)
transfers['#'] = maxval
altmap = createAltMap(rest, transfers, maxval)

#print altmap_w, altmap_h
#print transfers
print '<frames>'
print '<frame id="1">'
print '  <alt id="2" type="test" w="'+str(altmap_w)+'" h="'+str(altmap_h)+'" >'
print maptostring(altmap)
print '  </alt>'

smpath = [x for x in lines if x.find('PATH: G smoothed path:')>=0]
start_point=""
goal_point=""
if len(smpath)>0:
	path_line = smpath[0]
	path_line = path_line[path_line.find('PATH: G smoothed path:')+len('PATH: G smoothed path: '):]
	print '  <path id="3" type="result">'
	pid = 1
	ppp = [pp for pp in path_line.split(' ') if pp.strip()!='']
	last_id = len(ppp)
	for p in ppp:
		x,y = p[1:-1].split(',')
		if pid==1:
			start_point = '  <point id="4" type="start">\n'+'     '+x+","+y+'\n'+'    </point>'
		if pid==last_id:
			goal_point = '  <point id="5" type="goal">\n'+'     '+x+","+y+'\n'+'    </point>'
		print '    <point id="'+str(pid)+'">'
		print '       '+x+','+y
		print '    </point>'
		pid+=1
	print '  </path>'
	print start_point
	print goal_point
print '</frame>'
print '</frames>'

#print tostring(altmap)

#print 'END'
