unitsize(10mm);

real[] prm = {5, 8, 0, 2, 9, 7, 4, 3, 6, 12, 13, 11, 15, 1, 10, 17, 14, 16};

label(scale(1.5)*"Sender", (1.5, 10.5));
label(scale(1.5)*"Receiver", (7.5, 10.5));

filldraw(box((0,0), (3,10)), gray(0.9), black);

filldraw(box((5,0), (10,10)), gray(0.9), black);

// Sender arrows
for (real y=0.75; y<9.6; y+=0.5) {
  draw((1.0,y)--(2.0,y), gray(0.5), Arrow);
  draw((5.5,y)--(8.8,y), gray(0.5), Arrow);

  // Sender neurons
  filldraw(circle((1.0,y), 0.2), gray(0.5), black);

  // Reveiver neurons
  filldraw(circle((9.0,y), 0.2), gray(0.5), black);
}

// Sender data array
filldraw(box((2.0,0.5),(2.5,9.5)), gray(0.7), black);
for (real y=0.5; y<9.6; y+=0.5) {
  draw((2.0,y)--(2.5,y));
}

// Receiver data array
filldraw(box((5.5,0.5),(6,9.5)), gray(0.7), black);
for (real y=0.5; y<9.6; y+=0.5) {
  draw((5.5,y)--(6,y));
}

// Remapping arrows
for (int i=0; i<18; ++i)
  draw((2.5,0.75+i*0.5){right}..{right}(5.5,0.75+prm[i]*0.5), Arrow);

// Indicate processor borders
draw((-0.2,2.5)--(3.2,2.5), dashed);
draw((-0.2,5.0)--(3.2,5.0), dashed);
draw((-0.2,7.5)--(3.2,7.5), dashed);

draw((4.8,3.5)--(10.2,3.5), dashed);
draw((4.8,6.5)--(10.2,6.5), dashed);
