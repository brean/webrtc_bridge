#!/bin/bash
xhost +local:root
docker compose up viz
xhost -local:root