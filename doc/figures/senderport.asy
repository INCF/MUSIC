unitsize(10mm);

fill(box((2,0), (3,10)), gray(0.8));
draw(box((0,0), (3,10)));

// Sender arrows
for (real y=0.75; y<9.6; y+=0.5)
  draw((1.0,y)--(2.0,y), gray(0.5), Arrow);

// Sender data array
filldraw(box((0.5,0.5),(1.0,9.5)), gray(0.9), black);
for (real y=0.5; y<9.6; y+=0.5)
  draw((0.5,y)--(1.0,y));

label(rotate(90)*"Distributed sender data", (0.26, 5.0));

draw((-0.2,2.5)--(3.2,2.5), dashed);
draw((-0.2,5.0)--(3.2,5.0), dashed);
draw((-0.2,7.5)--(3.2,7.5), dashed);

// Port
filldraw(box((3.0,0.5),(3.5,9.5)), white, black);
label(rotate(90)*"Output Port", (3.24, 5.0));
