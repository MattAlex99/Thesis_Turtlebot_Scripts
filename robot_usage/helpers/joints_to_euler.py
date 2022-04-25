#! /usr/bin/env python





def joints_to_euler(j1,j2,j3,j4):
	roll=0
	pitch=j1
	yaw= j2+j3+j4
	return [roll, pitch, yaw]




