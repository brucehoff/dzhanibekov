# dzhanibekov
Simulation of the Dzhanibekov effect

Source code from Peter McGavin, who says,

"Written for Linux, it needs OpenGL dev libs and the 
commercial NAG Fortran library. In place of the latter, 
it could easily be modified to for NetLib RKSUITE routines."

http://homepages.paradise.n­et.nz/tmcgavin/box.c

The project is configured as an automated build in DockerHub.  To run:
```
docker run --rm -it -p 32800:5900 -e HOME=/ brucehoff/dzhanibekov
```

Then connect as explained [here](https://stackoverflow.com/questions/16296753/can-you-run-gui-apps-in-a-docker-container), e.g., on a Mac:
- type 'docker ps' to get the IP address and port for the running container, e.g., 0.0.0.0:32800
- go to the Finder
- click Command+K
- enter vnc://0.0.0.0:32800 (or whatever 'docker ps' returned in the first step)
- when prompted for a password, enter the value hardcoded in the accompanying Dockerfile, 1234


