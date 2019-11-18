FROM arm32v6/alpine

RUN apk update
RUN apk add alpine-sdk wireless-tools wireless-tools-dev
RUN apk add hostapd wpa_supplicant dnsmasq iw
RUN apk add python2
RUN apk add iptables

#RUN mkdir -p /etc/wpa_supplicant
#COPY ./configs/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant_RPi_access_point.conf

RUN mkdir -p /configs
COPY configs/hostapd.conf /configs
COPY configs/dnsmasq.conf /configs

RUN mkdir -p /scripts
COPY ./scripts/start_RPi_access_point.py /scripts/start_RPi_access_point.py
RUN chmod u+x /scripts/start_RPi_access_point.py
#RUN python /scripts/start_RPi_access_point.py

WORKDIR /scripts
ENTRYPOINT python /scripts/start_RPi_access_point.py
#ENTRYPOINT /bin/sh #uncomment for debugging and comment out line above
