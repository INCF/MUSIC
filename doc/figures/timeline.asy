import markers;

unitsize(10mm);

real Apos = 6;
real Bpos = 2;

real Alag = 1.5;
real Blag = 0.0;

real Aticks[] ={3, 5, 7, 9, 11, 13};
real Bticks[] ={3, 7, 11};

label(scale(2)*"A", (0, Apos), W);
label(scale(2)*"B", (0, Bpos), W);

// Application A ticks
for (int i=0; i<Aticks.length; ++i)
  fill(box((Aticks[i]-0.2, Apos-1), (Aticks[i]+0.2, Apos+1)), gray(0.8));

// Application B ticks
for (int i=0; i<Bticks.length; ++i)
  fill(box((Bticks[i]-0.2, Bpos-1), (Bticks[i]+0.2, Bpos+1)), gray(0.8));

void transfer(int a, int b) {
  draw("delay", (Bticks[b],Bpos+0.8)--(Bticks[b]-2.5,Bpos+0.8),
       S, marker(scale(2)*dotframe));
  draw((Aticks[a],Apos-1){down}..{down}(Bticks[b],Bpos+1), Arrow);
  draw((Bticks[b]-2.5+1.5,Apos-1){down}..{down}(Bticks[b]-2.5,Bpos+0.8),
     blue+dashed, Arrow);
}

transfer(0, 0);
transfer(2, 1);
transfer(4, 2);


draw(box((Alag,Apos-1), (Alag+13,Apos+1)));
draw(box((Blag,Bpos-1), (Blag+13,Bpos+1)));

draw("Wallclock Time", (0,0)--(15,0), Arrow);
