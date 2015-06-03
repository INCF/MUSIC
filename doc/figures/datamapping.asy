unitsize(10mm);

label(scale(1.5)*"Sender", (1.5, 10.5));
label(scale(1.5)*"Receiver", (11.5, 10.5));

fill(box((2,0), (3,10)), gray(0.8));
draw(box((0,0), (3,10)));

fill(box((10,0), (11,10)), gray(0.8));
draw(box((10,0), (13,10)));

// Sender arrows
for (real y=0.75; y<9.6; y+=0.5)
  draw((1.0,y)--(2.0,y), gray(0.5), Arrow);

// Sender data array
filldraw(box((0.5,0.5),(1.0,9.5)), gray(0.9), black);
for (real y=0.5; y<9.6; y+=0.5)
  draw((0.5,y)--(1.0,y));

label(rotate(90)*"Distributed sender data", (0.26, 5.0));

// Receiver arrows
for (real y=0.75; y<9.6; y+=0.5)
  draw((11.0,y)--(12.0,y), gray(0.5), Arrow);

// Receiver data array
filldraw(box((12.0,0.5),(12.5,9.5)), gray(0.9), black);
for (real y=0.5; y<9.6; y+=0.5)
  draw((12.0,y)--(12.5,y));

label(rotate(-90)*"Distributed receiver data", (12.74, 5.0));

draw((-0.2,2.5)--(3.2,2.5), dashed);
draw((-0.2,5.0)--(3.2,5.0), dashed);
draw((-0.2,7.5)--(3.2,7.5), dashed);

draw((9.8,3.5)--(13.2,3.5), dashed);
draw((9.8,6.5)--(13.2,6.5), dashed);

// Fan-in
draw((3,0.5){right}..tension 1.5 ..{right}(5.5,5.0));
draw((3,2.5){right}..{right}(5.5,5.0));
draw((3,5.0){right}..{right}(5.5,5.0));
draw((3,7.5){right}..{right}(5.5,5.0));
draw((3,9.5){right}..tension 1.5 ..{right}(5.5,5.0));

// Fan-out
draw((7.5,5.0){right}..tension 1.5 ..{right}(10,0.5));
draw((7.5,5.0){right}..{right}(10,3.5));
draw((7.5,5.0){right}..{right}(10,6.5));
draw((7.5,5.0){right}..tension 1.5 ..{right}(10,9.5));

draw((5.4,5.0)--(7.6,5.0), black+1, Arrow(position=0.8));
draw((5.5, 4.5)--(6.0,5.5));
label("Width", (6.0, 6.0));
