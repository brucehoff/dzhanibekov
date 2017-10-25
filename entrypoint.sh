# from https://stackoverflow.com/questions/6281998/can-i-run-glu-opengl-on-a-headless-server#8961649
Xvfb :5 -screen 0 1600x1000x24 &
export DISPLAY=:5
x11vnc -forever -usepw &
./box 

