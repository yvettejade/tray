f= open("movement.txt","r")
w= open("final.txt","w")
line=f.readline()
rf=[]
lf=[]
rb=[]
lb=[]
li=[]
ri=[]
lt=[]
tl=[]
counter=0;

while line:
	if line[0:2]=="RF":
		counter+=1
		rf.append(line[3:len(line)-1])
	if line[0:2]=="LF":
		lf.append(line[3:len(line)-1])
	if line[0:2]=="RB":
		rb.append(line[3:len(line)-1])
	if line[0:2]=="LB":
		lb.append(line[3:len(line)-1])
	if line[0:2]=="LI":
		li.append(line[3:len(line)-1])
	if line[0:2]=="RI":
		ri.append(line[3:len(line)-1])
	if line[0:2]=="LT":
		lt.append(line[3:len(line)-1])
	if line[0:2]=="TL":
		tl.append(line[3:len(line)-1])

	print(line[0:2])
	line=f.readline()

rfString=""
for index in rf:
	rfString+=index
	rfString+=", "

w.write(str(counter))
w.write("\n")
w.write("RF")
w.write("\n")
w.write(rfString);
w.write("\n")

rbString=""
for index in rb:
	rbString+=index
	rbString+=", "
w.write("RB")
w.write("\n")
w.write(rbString);
w.write("\n")

lfString=""
for index in lf:
	lfString+=index
	lfString+=", "
w.write("LF")
w.write("\n")
w.write(lfString);
w.write("\n")

lbString=""
for index in lb:
	lbString+=index
	lbString+=", "
w.write("LB")
w.write("\n")
w.write(lbString);

riString=""
for index in ri:
	riString+=index
	riString+=", "
w.write("RI")
w.write("\n")
w.write(lbString);

liString=""
for index in li:
	liString+=index
	liString+=", "
w.write("LI")
w.write("\n")
w.write(lbString);

ltString=""
for index in lt:
	ltString+=index
	ltString+=", "
w.write("LT")
w.write("\n")
w.write(lbString);

tlString=""
for index in tl:
	tlString+=index
	tlString+=", "
w.write("TL")
w.write("\n")
w.write(lbString);