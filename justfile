default:
	just --list

dev:
	pio run -t compiledb

build:
	pio run

fmt:
	trunk fmt -a

check:
	trunk check -a

qa: dev fmt check

conn:
	picocom -b 115200 /dev/ttyACM0
