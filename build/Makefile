build:
	mkdir -p ../classes
	javac ../src/Z80/*.java ../src/*.java -d ../classes

jar:
	mkdir -p ../bin
	jar vcmf manifest ../bin/J_ZX81.jar -C ../classes . ../data

run:
	java -cp ../classes:.. J_ZX81
