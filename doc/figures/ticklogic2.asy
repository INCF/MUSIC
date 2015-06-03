import markers;

unitsize(10mm);

label("Sender", (5, 8.5));
label("MUSIC", (0, 3), NE);
label("MPI", (0, 4.5), NE);
label("Receiver", (5, 1.5));

draw(box((10,8), (0,9)), dashed);
filldraw(box((10,3), (0,7)), gray(0.8), dotted);
draw(box((10,1), (0,2)), dashed);
filldraw(box((10,5.5), (0,4.5)), gray(0.5));

draw("Simulated Time", (0,0)--(10,0), Arrow);

// Senders ticks
int i=1;
for (real t=2.5; t<10; t+=5, ++i)
  draw(format("$s_{%d}$", i), (t, 8)--(t, 7), Arrow);

// Receivers ticks
int i=1;
for (real t=1; t<10; t+=2, ++i)
  draw(format("$r_{%d}$", i), (t, 3)--(t, 2), Arrow);

void transfer(real s1, real s2, real r1, real ts) {
  draw((s1,6.8)--(s2,6.8), marker(scale(2)*dotframe));
  draw((r1,6.8){down}..{down}(s2,5.5), Arrow);
  draw((s2,5.5){down}..{down}(s2-ts,4.5), blue, Arrow);
  draw((s2-ts,4.5){down}..{down}(r1,3.2), Arrow);
  draw((r1,3.1), marker(scale(2)*dotframe));
}

// Transfers
transfer(2.5, 7.5, 3, 4.5);
transfer(2.5, 7.5, 5, 4.5);
transfer(2.5, 7.5, 7, 4.5);
