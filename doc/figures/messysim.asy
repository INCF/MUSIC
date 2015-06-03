unitsize(10mm);

pair p11=(2,3);
pair p12=(1.5,2.5);
pair p13=(1,2);

pair p21=(6,2);
pair p22=(6.5,1.5);
pair p23=(7,1);

pair p31=(5,5);
pair p32=(5.3,5.1);
pair p33=(5.6,5.2);
pair p34=(5.9,5.3);
pair p35=(6.2,5.4);

void application(pair pos) {
  filldraw(box(pos-(1,1), pos+(1,1)), lightgray, black);
}

void flow(pair a, pair adir, int aoffs,
	  pair b, pair bdir, int boffs) {
  pair ao = (adir.y, -adir.x)*0.1*aoffs;
  pair bo = (-bdir.y, bdir.x)*0.1*boffs;
  draw(a+adir+ao{adir}..{-bdir}b+bdir+bo, black, Arrow);
}


application(p11);
application(p12);
application(p13);

application(p21);
application(p22);
application(p23);

application(p31);
application(p32);
application(p33);
application(p34);
application(p35);

flow(p11, E, -2, p21, W, -2);
flow(p11, E, 0, p22, W, -2);
flow(p11, E, 2, p23, W, -2);
flow(p12, E, -2, p21, W, 0);
flow(p12, E, 0, p22, W, 0);
flow(p12, E, 2, p23, W, 0);
flow(p13, E, -2, p21, W, 2);
flow(p13, E, 0, p22, W, 2);
flow(p13, E, 2, p23, W, 2);

flow(p11, N, -4, p31, W, -2);
flow(p11, N, -2, p32, W, -2);
flow(p11, N, 0, p33, W, -2);
flow(p11, N, 2, p34, W, -2);
flow(p11, N, 4, p35, W, -2);
flow(p12, N, -4, p31, W, 0);
flow(p12, N, -2, p32, W, 0);
flow(p12, N, 0, p33, W, 0);
flow(p12, N, 2, p34, W, 0);
flow(p12, N, 4, p35, W, 0);
flow(p13, N, -4, p31, W, 2);
flow(p13, N, -2, p32, W, 2);
flow(p13, N, 0, p33, W, 2);
flow(p13, N, 2, p34, W, 2);
flow(p13, N, 4, p35, W, 2);

flow(p21, N, 3, p31, S, 2);
flow(p21, N, 4, p32, S, 2);
flow(p21, N, 5, p33, S, 2);
flow(p21, N, 6, p34, S, 2);
flow(p21, N, 7, p35, S, 2);
flow(p22, N, 3, p31, S, 4);
flow(p22, N, 4, p32, S, 4);
flow(p22, N, 5, p33, S, 4);
flow(p22, N, 6, p34, S, 4);
flow(p22, N, 7, p35, S, 4);
flow(p23, N, 3, p31, S, 6);
flow(p23, N, 4, p32, S, 6);
flow(p23, N, 5, p33, S, 6);
flow(p23, N, 6, p34, S, 6);
flow(p23, N, 7, p35, S, 6);

flow(p31, S, 2, p21, N, 8);
flow(p31, S, 4, p22, N, 8);
flow(p31, S, 6, p23, N, 8);
flow(p32, S, 2, p21, N, 6);
flow(p32, S, 4, p22, N, 6);
flow(p32, S, 6, p23, N, 6);
flow(p33, S, 2, p21, N, 4);
flow(p33, S, 4, p22, N, 4);
flow(p33, S, 6, p23, N, 4);
flow(p34, S, 2, p21, N, 2);
flow(p34, S, 4, p22, N, 2);
flow(p34, S, 6, p23, N, 2);
flow(p35, S, 2, p21, N, 0);
flow(p35, S, 4, p22, N, 0);
flow(p35, S, 6, p23, N, 0);

label("LGN", p13);
label("NEST", p23);
label("Neuron", p35);
