FROM lapidarioz/docker-cpp-opencv3-glut

WORKDIR /
RUN wget http://www.netlib.org/ode/rksuite/rksuitec++.zip && unzip rksuitec++.zip
WORKDIR /RksuiteTest
RUN gcc -c -fPIC rksuite.cpp -o rksuite.o -w
RUN gcc -shared -o librksuite.so rksuite.o
RUN readelf -Ws librksuite.so
RUN cp librksuite.so /usr/lib/
RUN cp rksuite.h /usr/include

COPY . /usr/src/box
WORKDIR /usr/src/box
# This is the build command from tmcgavin
#RUN gcc -o box box.cpp -DUNIX -O2 -w -Wall -s -lglut -lGLU -lGL -L/usr/X11R6/lib -lXi -lXmu -lX11 -lnag -lg2c -lm
RUN gcc -o box box.cpp -DUNIX -O2 -w -Wall -s -lglut -lGLU -lGL -L/usr/X11R6/lib -lXi -lXmu -lX11 -lm -lrksuite
CMD ["./box"]
