INC      = /opt/gurobi702/linux64/include/
CPP      = g++
CARGS    = -m64 -g
CPPLIB   = -L /opt/gurobi702/linux64/lib/ -lgurobi_c++ -lgurobi70
JFLAG    = -d . $(CLASSDIR)

stretch:
	$(CPP) $(CARGS) stretch.cpp -o stretch -I$(INC) $(CPPLIB) -lm --std=c++14
	./stretch

clean:
	rm *.o
