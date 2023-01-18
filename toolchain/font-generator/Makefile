
FONT_FILE=font_24x40.png
#FONT_FILE=asd.txt


all: font-generator

font-generator: main.cpp
	$(CXX) main.cpp -o font-generator -lpng16 --std=c++17

font-deps: font-generator $(FONT_FILE)

font: font-deps
	./font-generator $(FONT_FILE) $(OUTPUT)
