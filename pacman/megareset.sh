#!/usr/bin/env bash

hostname=$1

curl -m 10 -s -w '%{http_code}' -XPOST http://$hostname/console/reset
