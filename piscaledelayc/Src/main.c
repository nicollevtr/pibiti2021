void init()
{
*((int*)(0x40023800 + 0x30)) = 0x00000008;
*((int*)(0x40020C00 + 0x00)) = 0x01000000;
*((int*)(0x40020C00 + 0x04)) = 0x00000000;
*((int*)(0x40020C00 + 0x08)) = 0x00000000;
*((int*)(0x40020C00 + 0x0C)) = 0x00000000;

int on = 0x1000;
int off = 0x0000;
int i;
int * odr = (int*)(0x40020C00 + 0x14);

loop:
	i = 800000;
	*odr = on;
	while(i--);
	*odr = off;
	i = 800000;
	while(i--);
	goto loop;
}

void (*isr[])() __attribute__ ((section (".vector")))
={
(void (*)())0x20020000,
init
};
