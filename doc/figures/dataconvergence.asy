unitsize(10mm);

pair p1=(1,2);
pair p2=(6,2.1);

void application(int n, pair pos) {
  for (int i=n-1; i>=0; --i) {
    filldraw(box(pos-(1,1)+i*(0.1, 0.1), pos+(1,1)+i*(0.1, 0.1)),
	     lightgray, black);
  }
}

application(5, p1);

draw("2.6\,Tb/s", p1+(1.2,0.2){right}..{right}p2+(-0.9,0.1), black+1, Arrow);
draw("2.6\,Gb/s", p2+(1.2,0.1){right}..{right}p2+(3.2,0.1), black+1, Arrow);

application(3, p2);

label("Simulator", p1);
label("EEG", p2);
