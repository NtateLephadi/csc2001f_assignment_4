SRCDIR = src
BINDIR = bin
DOCDIR = doc
JAVAC = javac
JFLAGS = -g -d $(BINDIR) -cp $(BINDIR)

vpath %.java $(SRCDIR)
vpath %.class $(BINDIR)

# define general build rule for java sources
.SUFFIXES:  .java  .class

.java.class:
	$(JAVAC)  $(JFLAGS)  $<

#default rule - will be invoked by make

all: SimulatorOne.class

default: $(ALL)
	javac -d $(BINDIR)

# Rules for generating documentation
doc:
	javadoc -d $(DOCDIR) $(SRCDIR)/*.java

# run HashValuesGenerator.java
simulatorone:
	java -cp $(BINDIR) SimulatorOne

# clean docs and class files
clean:
	@rm -f  $(BINDIR)/*.class $(SRCDIR)/*.class
	@rm -Rf doc
