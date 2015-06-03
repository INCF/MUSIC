unitsize(10mm);

fill(box((10,0), (11,10)), gray(0.8));
draw(box((10,0), (13,10)));

// Receiver arrows
for (real y=0.75; y<9.6; y+=0.5)
  draw((11.0,y)--(12.0,y), gray(0.5), Arrow);

// Receiver data array
filldraw(box((12.0,0.5),(12.5,9.5)), gray(0.9), black);
for (real y=0.5; y<9.6; y+=0.5)
  draw((12.0,y)--(12.5,y));

label(rotate(-90)*"Distributed receiver data", (12.74, 5.0));

draw((9.8,3.5)--(13.2,3.5), dashed);
draw((9.8,6.5)--(13.2,6.5), dashed);

// Port
filldraw(box((9.5,0.5),(10.0,9.5)), white, black);
label(rotate(-90)*"Input Port", (9.74, 5.0));
