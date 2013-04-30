PROJECT =  cits3230project
HEADERS =  ap.h dll_ethernet.h dll_shared.h dll_wifi.h mapping.h mobile.h network.h walking.h
OBJ     =  ap.o dll_ethernet.o dll_wifi.o mapping.o mobile.o project.o walking.o
        

C99     =  gcc -std=c99
CFLAGS  =  -Wall -pedantic -Werror 

$(PROJECT) : $(OBJ)
		$(C99) $(CFLAGS) -o $(PROJECT) $(OBJ)

%.o : %.c $(HEADERS)
	$(C99) $(CFLAGS) -c $<

clean:
	rm -rf f? *.o *.cnet result.*

