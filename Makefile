all:
	mex -g atlas.cpp kin-dyn.cpp

clean:
	rm -f *.mexa64