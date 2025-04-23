#!/bin/bash
terminator -l webrtc_stream -g terminator.conf
docker compose kill viz webrtc-bridge cam
docker compose down