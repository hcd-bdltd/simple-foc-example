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

upload:
	pio run -t upload

conn:
	tio /dev/ttyACM0
