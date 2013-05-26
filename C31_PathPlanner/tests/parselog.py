#!/usr/bin/python

def readlines():
	lines=[]
	import fileinput
	for line in fileinput.input():
		lines.append(line[:-1])
	return lines

lines = readlines()


def ProcFrame(fid, lines):
	def printGrid(eid, lines):
		this_map=False
		line_num=0
		w=0
		h=0
		grid=[]
		#print "\n".join(lines)
		for line in lines:
			if line.find('Calculated path')>=0: this_map = True
			if this_map:
				if len(line.strip())==0:
					break
				line_num+=1
				if line_num==2:
					w=int(line.strip().split(' ')[-1])+1
				if line_num==3:
					h=int(line.strip().split(' ')[0])+1
				if line_num > 2:
					grid.append( line.strip().split(' ')[1:] )
		#print "\n".join([ " ".join([c for c in r]) for r in grid])
		def C(x):
			if x=='-': return 250
			if x=='.': return 255
			if x=='B': return 0
			return 200
		grid = reversed([ '      '+','.join([ str(C(col)) for col in row if len(col)>0]) for row in grid])
		print '   <map id="'+str(eid)+'" w="'+str(w)+'" h="'+str(h)+'">'
		print '\n'.join(grid)
		print '   </map>'
		
	def printPublishPath(eid, lines):
		this_path=False
		line_num=0
		points=[]
		for line in lines:
			if line.find('PUBLISH CALCULATED GLOBAL PATH')>=0:
				this_path = True
				continue
			if this_path:
				if line.strip().find('PATH:')<0:
					break
				L='PATH: cell:'
				M='  ->  '
				x,y=line[line.find(L)+len(L):line.find(M)].split(',')
				points.append( (x,y) )
		print '   <path id="'+str(eid)+'" type="result">'
		i=0
		for p in points:
			print '       <point id="'+str(i)+'">'
			print '          '+p[0]+','+p[1]
			print '       </point>'
			i+=1
		print '   </path>'
	
	def printPath(eid, lines):
		LINE='Calculated path: Path#'
		smpath = [x for x in lines if x.find(LINE)>=0]
		start_point=""
		goal_point=""
		if len(smpath)>0:
			path_line = smpath[0]
			path_line = path_line[path_line.find(LINE)+len(LINE)+3:]
			print '  <path id="'+str(eid)+'" type="result">'
			pid = 1
			ppp = [pp for pp in path_line.split(' ') if pp.strip()!='']
			last_id = len(ppp)
			for p in ppp:
				x,y = p[1:-1].split(',')
				if pid==1:
					start_point = '  <point id="'+str(eid+1)+'" type="start">\n'+'     '+x+","+y+'\n'+'    </point>'
				if pid==last_id:
					goal_point = '  <point id="'+str(eid+2)+'" type="goal">\n'+'     '+x+","+y+'\n'+'    </point>'
				print '    <point id="'+str(pid)+'">'
				print '       '+x+','+y
				print '    </point>'
				pid+=1
			print '  </path>'
			print start_point
			print goal_point
	
	def printStartAndGoals(eid,lines):
		line = [ x for x in lines if x.find('PathPlanning::plan : searchPath')>=0]
		if len(line)<1:
			return
		l = line[0]
		s=l.find('start=')
		if s>=0: s+=len('start=')
		f=l.find('finish=')
		if f>=0: f+=len('finish=')
		ps = l[s:l.find(' ',s)][:-1]
		pf = l[f:l.find(' ',f)][:-1]
		print '   <point id="'+str(eid)+'" type="start">'
		print '          '+ps
		print '   </point>'
		print '   <point id="'+str(eid+1)+'" type="goal">'
		print '          '+pf
		print '   </point>'
	
	
	print '<frame id="'+str(fid)+'">'
	printGrid(10,lines)
	printPublishPath(30,lines)
	printStartAndGoals(40,lines)
	print '</frame>'

	
def splitFrames(lines):
	frames=[]
	frame=[]
	for line in lines:
		if line.find('PathPlanning::plan : searchPath')>=0:
			frames.append(frame)
			frame=[]
			frame.append(line)
		else:
			frame.append(line)
	frames.append(frame)
	return frames
	
	
print '<frames>'
i=0
frames = splitFrames(lines)
for frame in frames:
	ProcFrame(i, frame)
	if i==2: break
	i+=1
print '</frames>'

