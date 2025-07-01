#!/bin/bash
terminator -l webrtc_stream -g terminator_example1.conf
docker compose kill viz webrtc-bridge cam
docker compose down