FROM resin/rpi-raspbian:stretch

RUN [ "cross-build-start" ]
RUN apt-get update && apt-get install build-essential mosquitto
ADD packages ./packages
RUN dpkg -i ./packages/*.deb; true
RUN apt-get install -f
RUN [ "cross-build-end" ]