# parameters
ARG REPO_NAME="duckiebot"
ARG DESCRIPTION="Color detector for the dockiebot"
ARG MAINTAINER="Anna Hakhverdyan (<annahakhverdyan98@gmail.com>)"

FROM duckietown/dt-duckiebot-interface:daffy-arm32v7 
WORKDIR /assignment1

COPY requirements.txt ./
RUN pip install -r requirements.txt

COPY color_detector.py .
CMD python3 ./color_detector.py