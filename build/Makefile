build:
	mkdir ../classes
	javac ../src/Z80/*.java ../src/*.java -d ../classes

jar:
	jar vcmf manifest J_ZX81.jar -C ../classes . ../data

run:
	java -cp ../classes:.. J_ZX81
