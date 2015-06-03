unitsize(10mm);

pair p1=(1,2);
pair p2=(6,1);
pair p3=(5,5);

void application(int n, pair pos) {
  for (int i=n-1; i>=0; --i) {
    filldraw(box(pos-(1,1)+i*(0.1, 0.1), pos+(1,1)+i*(0.1, 0.1)),
	     lightgray, black);
  }
}

application(5, p1);

draw(p1+(0.2,1.2){up}..{right}p3+(-0.9,0.1), black+1, Arrow);
draw(p1+(1.2,0.2){right}..{right}p2+(-0.9,0.1), black+1, Arrow);

application(3, p2);

pair d=(0.3,0);

draw(p3+(0.1,-0.9)-d{down}..{down}p2+(0.1,1.1)-d, black+1, Arrow);
draw(p2+(0.1,1.1)+d{up}..{up}p3+(0.1,-0.9)+d, black+1, Arrow);

application(3, p3);

label("Appl. $A$", p1);
label("Appl. $B$", p2);
label("Appl. $C$", p3);
