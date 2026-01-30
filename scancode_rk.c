#include "init.h"
#include "bsp/board.h"
#include "tusb.h"
#include "scancode_rk.h"

uint8_t tab_key[128] = {0x00};     // таблица нажатых клавиш
uint8_t tab_key_old[128] = {0x00}; // таблица предыдушего нажатия клавиш

//----------------------------------------------------------
// table usb
//--------------------------------------------------------
// В основной таблице каждому скан-коду IBM-клавиатуры
// соответствует один байт, который содержит информацию
// о номере колонки и номере строки, в которой будет
// имитироватся замыкание контакта клавиатуры Спектрума.
// d6 сигнализирует о дополнительном нажатии Symbol Shift.
// d7 о нажатии Caps Shift.
// Для клавиш IBM клавиатуры, которые в зависимости от
// нажатия Shift имеют разные коды, предусмотрено перек-
// лючение таблицы на дополнительную, признаком этого
// является d7,d6=1.
// Пропущенные скан-коды можно забить любым кодом.
// Незадействованные скан-коды заполняются кодом 0FFH.
// Поскольку таблица жестко связана со скан-кодами,
// нельзя ни пропускать, ни добавлять в нее строки.
// Дополнительную таблицу можно расширять в сторону
// увеличения практически до 63 строк. Но начало
// этой таблицы тоже жестко определено (адрес 480H).
// Для примера:
// Скан код клавиши TAB для IBM-клавиатуры равен 0DH
// в строке номер 13 от начала таблицы видим:
//	DB	Kl_TAB,Kl_TAB		;0Dh	 Tab
// Поскольку клавиша TAB судя по матрице клавиатуры
// располагается на пересечении строки A1 и колонки D0,
// то определяем Kl_TAB в виде константы:
//Kl_TAB	EQU	A1+D0	;TAB
// A1 и D0 это тоже константы:
//A1	EQU	008H	;1*8
//D0	EQU	000H	;0
// В принципе можно было бы записать и так:
//	DB	A1+D0,A1+D0		;0Dh	 Tab
// Или еще проще:
//	DB	8+0,8+0			;0Dh	 Tab
// но это не очень наглядно.
//--------------------------------------------------------
//   Матрица клавиатуры
// -----------------------------------
//    | D7  D6  D5  D4  D3  D2  D1  D0
// ---|-------------------------------
// A7 |SPC  ^   ]   \   [   Z   Y   X
// A6 | W   V   U   T   S   R   Q   P
// A5 | O   N   M   L   K   J   I   H
// A4 | G   F   E   D   C   B   A   @
// A3 | /   .   =   ,   ;   :   9   8
// A2 | 7   6   5   4   3   2   1   0
// A1 | v  ->   ^  <-  ЗАБ ВК  ПС  TAB
// A0 |F5  F4  F3  F2  F1  AP2 CTP  \
// -----------------------------------
// биты данных сканирования (d2..d0) [номер колонки 0..7]
#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
// биты адреса сканирования (d5..d3) [номер строки *8]
#define A0 0x00
#define A1 0x08
#define A2 0x10
#define A3 0x18
#define A4 0x20
#define A5 0x28
#define A6 0x30
#define A7 0x38
// Префиксные биты (d7..d6)
#define Ctrl	0x80	//флаг Ctrl   Bit7=1
#define Shift	0x40	//флаг Shift  Bit6=1
#define AltTb	0xC0	//флаг доп.таблицы
// скан-коды основных клавиш
#define Kl_SL	A0+D0	// Home
#define Kl_CTP 	A0+D1	// СТР
#define Kl_AP2 	A0+D2	// АР2
#define Kl_F1	A0+D3
#define Kl_F2	A0+D4
#define Kl_F3	A0+D5
#define Kl_F4	A0+D6
#define Kl_F5	A0+D7

#define Kl_TAB	A1+D0	//TAB
#define Kl_LF	A1+D1	//ПС
#define Kl_CR	A1+D2	//Enter
#define Kl_BS	A1+D3	//ЗАБ
#define Kl_LFT	A1+D4	//Влево
#define Kl_UP	A1+D5	//Вверх
#define Kl_RGT	A1+D6	//Вправо
#define Kl_DN	A1+D7	//Вниз

#define Kl_0    A2+D0	// 0
#define Kl_1    A2+D1	// 1
#define Kl_2    A2+D2	// 2
#define Kl_3    A2+D3   // 3
#define Kl_4    A2+D4	// 4
#define Kl_5    A2+D5	// 5
#define Kl_6    A2+D6	// 6
#define Kl_7    A2+D7	// 7

#define Kl_8	A3+D0	// 8
#define Kl_9	A3+D1	// 9
#define Kl_DVT	A3+D2	// : двоеточие
#define Kl_PLS	A3+D3	// ;/+ PLUS
#define Kl_ZPT	A3+D4	// , запятая
#define Kl_MNS	A3+D5	// =/- МИНУС
#define Kl_TCK	A3+D6	// . точка
#define Kl_DEV	A3+D7	// / деление

#define Kl_AMP	A4+D0	// @ амперсант
#define Kl_A	A4+D1
#define Kl_B	A4+D2
#define Kl_C	A4+D3
#define Kl_D	A4+D4
#define Kl_E	A4+D5
#define Kl_F	A4+D6
#define Kl_G	A4+D7

#define Kl_H	A5+D0
#define Kl_I	A5+D1
#define Kl_J	A5+D2
#define Kl_K	A5+D3
#define Kl_L	A5+D4
#define Kl_M	A5+D5
#define Kl_N	A5+D6
#define Kl_O	A5+D7

#define Kl_P	A6+D0
#define Kl_Q	A6+D1
#define Kl_R	A6+D2
#define Kl_S	A6+D3
#define Kl_T	A6+D4
#define Kl_U	A6+D5
#define Kl_V	A6+D6
#define Kl_W	A6+D7

#define Kl_X	A7+D0
#define Kl_Y	A7+D1
#define Kl_Z	A7+D2
#define Kl_BL	A7+D3	// [
#define Kl_OSL	A7+D4	// \_  
#define Kl_BR	A7+D5	// ]
#define Kl_KAV	A7+D6	// ^
#define Kl_SP	A7+D7	//Пробел

#define Kl_NOKEY 0xFF

#define Kl_Shift 0x20
#define Kl_Ctrl  0x40
#define Kl_RusLat 0x80
//----------------------------------------------------------------
// Таблица скан-кодов клавиш AT
// Четные байты для режима LAT - Scroll Lock не горит
// Нечетные байты для RUS - Scroll Lock горит.
//		это скан-код IBM vvv
uint8_t tab_kbd[] = {				//	 vvv - а это клавиша IBM
	0xFF,0xFF,				//00h
	0xFF,0xFF,				//01h
	0xFF,0xFF,				//02h
	0xFF,0xFF,				//03h
	Kl_A,Kl_F,				//04h	A
	Kl_B,Kl_I,				//05h	B
	Kl_C,Kl_S,				//06h	C
	Kl_D,Kl_W,				//07h	D
	Kl_E,Kl_U,				//08h	E
	Kl_F,Kl_A,				//09h	F
	Kl_G,Kl_P,				//0Ah	G
	Kl_H,Kl_R,				//0Bh	H
	Kl_I,Kl_BL,				//0Ch	I
	Kl_J,Kl_O,				//0Dh	J
	Kl_K,Kl_L,				//0Eh	K
	Kl_L,Kl_D,				//0Fh	L
	Kl_M,Kl_X,				//10h	M
	Kl_N,Kl_T,				//11h	N
	Kl_O,Kl_BR,				//12h	O
	Kl_P,Kl_Z,				//13h	P
	Kl_Q,Kl_J,				//14h	Q
	Kl_R,Kl_K,				//15h	R
	Kl_S,Kl_Y,				//16h	S
	Kl_T,Kl_E,				//17h	T
	Kl_U,Kl_G,				//18h	U
	Kl_V,Kl_M,				//19h	V
	Kl_W,Kl_C,				//1Ah	W
	Kl_X,Kl_KAV,			//1Bh	X
	Kl_Y,Kl_N,				//1Ch	Y
	Kl_Z,Kl_Q,				//1Dh	Z
	Kl_1,Kl_1,				//1Eh	1/!
	AltTb+6,AltTb+6,		//1Fh	2/@
	Kl_3,Kl_3,				//20h	3/#
	Kl_4,Kl_4,				//21h	4/$
	Kl_5,Kl_5,				//22h	5/%
	AltTb+8,AltTb+8,		//23h	6/^
	AltTb+9,AltTb+9,		//24h	7/&
	AltTb+1,AltTb+1,		//25h	8/*
	AltTb+2,AltTb+2,		//26h	9/(
	AltTb+3,AltTb+3,		//27h	0/)
	Kl_CR,Kl_CR,			//28h	ENTER
	Kl_AP2,Kl_AP2,			//29h 	ESC
	Kl_BS,Kl_BS,			//2Ah	BackSpace
	Kl_TAB,Kl_TAB,			//2Bh	Tab
	Kl_SP,Kl_SP,			//2Ch	SPACE
	AltTb+4,AltTb+4,		//2Dh	-/_
	AltTb+5,AltTb+5,		//2Eh	=/+
	Kl_BL,Kl_H,				//2Fh	[/{
	Kl_BR,Kl_BR,			//30h	]/}
	Kl_OSL,Kl_OSL,			//31h	\/|
	0xFF,0xFF,				//32h   #
	AltTb,Kl_V,				//33h	;/:
	AltTb+10,AltTb+10,		//34h	'/"
	AltTb+7,AltTb+7,		//35h	 `/~ 
	Kl_ZPT,Kl_B,			//36h	,/<
	Kl_TCK,Kl_AMP,			//37h	./>
	Kl_DEV,Kl_DEV,			//38h	//?
	0xFF,0xFF,				//39h	Caps Lock
	Kl_F1,Kl_F1,			//3Ah 	F1
	Kl_F2,Kl_F2,			//3Bh 	F2
	Kl_F3,Kl_F3,			//3Ch	F3
	Kl_F4,Kl_F4,			//3Dh	F4
	Kl_F5,Kl_F5,			//3Eh	F5
	Shift+Kl_F1,Shift+Kl_F1,//3Fh	F6
	Shift+Kl_F2,Shift+Kl_F2,//40h	F7
	Shift+Kl_F3,Shift+Kl_F3,//41h	F8
	Shift+Kl_F4,Shift+Kl_F4,//42h	F9
	Shift+Kl_F5,Shift+Kl_F5,//43h	F10
	0xFF,0xFF,				//44h	F11
	0xFF,0xFF,				//45h	F12
	0xFF,0xFF,				//46h	Print Screen -> RESET
	0xFF,0xFF,				//47h	Scroll Lock
	Kl_LF,Kl_LF,			//48h	Pause/Break -> WAIT
	A0+D1,A0+D1,			//49h	[Insert] - СТР
	//A0+D0,A0+D0   		//4Ah	[Home]
	Ctrl+Kl_LFT,Ctrl+Kl_LFT,//4Ah	[Home]
	//Ctrl+Kl_R,Ctrl+Kl_R	//4Bh	[PageUp]
	Ctrl+Kl_UP,Ctrl+Kl_UP,  //4Bh	[PageUp]
	//Ctrl+Kl_G,Ctrl+Kl_G	//4Ch	[Delete]
	Kl_F4,Kl_F4,			//4Ch	[Delete]
	//A1+D1,A1+D1			//4Dh	[End]
	Ctrl+Kl_RGT,Ctrl+Kl_RGT,//4Dh	[End]
	//A0+D6,A0+D6,	        //4Eh	[PageDown]
	Ctrl+Kl_DN,Ctrl+Kl_DN,	//4Eh	[PageDown]
	Kl_RGT,Kl_RGT,	        //4Fh	[Right]
	Kl_LFT,Kl_LFT,			//50h	[Left]
	Kl_DN,Kl_DN,			//51h	[Down]
	Kl_UP,Kl_UP,			//52h	[Up]
	0xFF,0xFF,		    	//53h	NumLock
	AltTb+13,AltTb+13,		//54h	[/]
	AltTb+12,AltTb+12,		//55h	[*]
	AltTb+14,AltTb+14, 		//56h 	[-]
	AltTb+11,AltTb+11,		//57h	[+]
	Kl_CR,Kl_CR,			//58h	[ENTER]
	AltTb+23,AltTb+23,  	//59h	[1]
	AltTb+17,AltTb+17,		//5Ah	[2]
	AltTb+21,AltTb+21,		//5Bh	[3]
	AltTb+24,AltTb+24,		//5Ch	[4]
	AltTb+18,AltTb+18,		//5Dh	[5]
	AltTb+19,AltTb+19,		//5Eh	[6]
	AltTb+25,AltTb+25,		//5Fh	[7]
	AltTb+20,AltTb+20,		//60h	[8]
	AltTb+22,AltTb+22,		//61h	[9]
	AltTb+15,AltTb+15,		//62h	[0]
	AltTb+16,AltTb+16,		//63h	[.]
};

//--------------------------------------------
// Таблица клавиш с двумя кодами:
// 1код - без Shift
// 2код -  с  Shift
uint8_t AltTab[]={
	Kl_PLS,Kl_DVT,			    // 0	33h	;/:
	Kl_8,Kl_DVT+Shift,		    // 1	25h	8/*
	Kl_9,Kl_8+Shift,		    // 2	26h	9/(
	Kl_0,Kl_9+Shift,		    // 3	27h	0/)
	Kl_MNS,Kl_BS+Shift,		    // 4	2Dh	-/_
	Kl_MNS+Shift, Kl_PLS+Shift,	// 5	2Eh	=/+
	Kl_2,Kl_AMP,			    // 6	2Eh	2/@
	Kl_AMP+Shift,Kl_KAV+Shift,	// 7	35h	 `/~ 
	Kl_6,Kl_KAV,			    // 8	23h	6/^
	Kl_7,Kl_6+Shift,			// 9	24h	7/&
	Kl_2+Shift,Kl_7+Shift,		//10	34h	'/"
	Kl_PLS+Shift,Kl_PLS+Shift,	//11	57h	[+]
	Kl_DVT+Shift,Kl_DVT+Shift,	//12	55h	[*]
	Kl_DEV,Kl_DEV,			    //13	54h	[/]
	Kl_MNS,Kl_MNS,			    //14	56h [-]
	Kl_0,Kl_0,			        //15	62h	[0]
	Kl_TCK,Kl_TCK,			    //16	63h	[.]
	Kl_2,Kl_2,			        //17	5Ah	[2]
	Kl_5,Kl_5,			        //18	5Dh	[5]
	Kl_6,Kl_6,			        //19	5Eh	[6]
	Kl_8,Kl_8,   		        //20	60h	[8]
	Kl_3,Kl_3,			        //21	5Bh	[3]
	Kl_9,Kl_9,			        //22	61h	[9]
	Kl_1,Kl_1,			        //23	59h	[1]
	Kl_4,Kl_4,			        //24	5Ch	[4]
	Kl_7,Kl_7,			        //25	5Fh	[7]
};

//--------------------------------------------
// Таблица клавиш с двумя кодами (без NumLock):
// 1код - без Shift
// 2код -  с  Shift
uint8_t AltTab2[]={
	Kl_PLS,Kl_DVT,			    // 0	33h	;/:
	Kl_8,Kl_DVT+Shift,		    // 1	25h	8/*
	Kl_9,Kl_8+Shift,		    // 2	26h	9/(
	Kl_0,Kl_9+Shift,		    // 3	27h	0/)
	Kl_MNS,Kl_BS+Shift,		    // 4	2Dh	-/_
	Kl_MNS+Shift, Kl_PLS+Shift,	// 5	2Eh	=/+
	Kl_2,Kl_AMP,			    // 6	2Eh	2/@
	Kl_AMP+Shift,Kl_KAV+Shift,	// 7	35h	 `/~ 
	Kl_6,Kl_KAV,			    // 8	23h	6/^
	Kl_7,Kl_6+Shift,			// 9	24h	7/&
	Kl_2+Shift,Kl_7+Shift,		//10	34h	'/"
	Kl_PLS+Shift,Kl_PLS+Shift,	//11	57h	[+]
	Kl_DVT+Shift,Kl_DVT+Shift,	//12	55h	[*]
	Kl_DEV,Kl_DEV,			    //13	54h	[/]
	Kl_MNS,Kl_MNS,			    //14	56h [-]
	A0+D1,A0+D1,		        //15	62h	[0] [Ins] = ПС
	Kl_F4,Kl_F4,			    //16	63h	[.] [Del] = F4
	Kl_DN,Kl_DN,			    //17	5Ah	[2] [Down]
	255,255,			        //18	5Dh	[5]
	Kl_RGT,Kl_RGT,		        //19	5Eh	[6] [Right]
	Kl_UP,Kl_UP,		        //20	60h	[8] [Up]
	Ctrl+Kl_DN,Ctrl+Kl_DN,      //21	5Bh	[3] [PgDn]
	Ctrl+Kl_UP,Ctrl+Kl_UP,      //22	61h	[9] [PgUp]
	Ctrl+Kl_RGT,Ctrl+Kl_RGT,    //23	59h	[1] [End]
	Kl_LFT,Kl_LFT,		        //24	5Ch	[4] [Left]
	Ctrl+Kl_LFT,Ctrl+Kl_LFT,    //25	5Fh	[7] [Home]
};
//********************************************
uint8_t rk_matrix[8] = {0};

#define KBD_MOD_SHIFT 0x71
#define KBD_MOD_ALT   0x70
#define KBD_MOD_CTRL  0x72
#define KBD_MOD_WIN   0x73

//---------------------------------------------------
void rk_scancode_s(uint8_t code)
{
    if (code & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT))
        tab_key[KBD_MOD_SHIFT] = 1; // left shift

	if (code & (KEYBOARD_MODIFIER_LEFTALT | KEYBOARD_MODIFIER_RIGHTALT))
        tab_key[KBD_MOD_ALT] = 1; // left alt     0000 0100

    if (code & (KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_RIGHTCTRL))
        tab_key[KBD_MOD_CTRL] = 1; // right ctrl

    if (code & (KEYBOARD_MODIFIER_LEFTGUI | KEYBOARD_MODIFIER_RIGHTGUI))
        tab_key[KBD_MOD_WIN] = 1; // win  l
}
//----------------------------------------------------------
bool ledCapsLock = false;
bool ledNumLock = true;
bool ledScrollLock = false;
///////////////////////////////////////////////////////////////////////////////////////////////
uint8_t leds = 0;
extern uint8_t kb_addr;
extern uint8_t kb_inst;

void kb_update_leds()
{
	leds = (ledScrollLock << 2) | (ledCapsLock<<1) | ledNumLock;
	tuh_hid_set_report(kb_addr, kb_inst, 0, HID_REPORT_TYPE_OUTPUT, &leds, sizeof(leds));
	tuh_task();
}


uint8_t rk_get_key(uint8_t code, bool bOn, bool *pbAltTab)
{
    //code = code & 0x7f; //???
	if (bOn)
	{
		do
		{
			if (code == KBD_MOD_ALT)				/*39 Caps Lock  */
				ledCapsLock = !ledCapsLock; // тригер  Caps lock
			else if (code == HID_KEY_NUM_LOCK)
				ledNumLock = !ledNumLock; // тригер  Caps lock
			else if (code == HID_KEY_SCROLL_LOCK)
				ledScrollLock = !ledScrollLock; // тригер  Caps lock
			else
				break;
			kb_update_leds();
		} while (false);
	}

	bool bShift = tab_key[KBD_MOD_SHIFT] != 0;
	code <<= 1; // умножение на 2
	if (code >= sizeof(tab_kbd))
		return 0xFF;
    if (ledCapsLock)
		++code;
	uint8_t key = tab_kbd[code];
	if (key == 0xFF)
		return key;
	if ((key & AltTb) == AltTb)
	{
		uint8_t s = (key - AltTb) * 2;
		if (bShift) ++s;
		key = (ledNumLock?AltTab:AltTab2)[s];
		if (pbAltTab)
			*pbAltTab = true;
	}
	return key;
}

extern volatile uint8_t kbdMatr[8];// = {255,255,255,255,255,255,255,255};
extern volatile uint8_t portC;// = 0xF0; 

void __noinline rk_key(uint8_t code, bool bOn) // клавиша нажата/отпущена
{
	// if (code>=KBD_MOD_ALT)
	// 	return;
	bool bAltTab = false;
	uint8_t key = rk_get_key(code, bOn, &bAltTab);
	if (key == Kl_NOKEY && code < KBD_MOD_ALT)
		return;
	uint8_t row = /* 7 -  */(key >> 3) & 7; // Row
	uint8_t col = key & 7;	 // Column
	bool bShift = tab_key[KBD_MOD_SHIFT] != 0;
	bool bCtrl = tab_key[KBD_MOD_CTRL] != 0;
	bool bRusLat = tab_key[KBD_MOD_ALT] != 0;
	
	if (bOn)
	{
		bShift = bShift || (key != Kl_NOKEY && (key & Shift) != 0);
		bCtrl = bCtrl || (key != Kl_NOKEY && (key & Ctrl)!=0);
	}
	portC = 0xE0 & ~((bCtrl ? Kl_Ctrl : 0) | (bShift ? Kl_Shift : 0) | (bRusLat ? Kl_RusLat : 0));
	if (bOn)
	{
		if (code < KBD_MOD_ALT)
			kbdMatr[row] &= ~(1 << col);
	}
	else
	{
		if (code < KBD_MOD_ALT)
			kbdMatr[row] |= (1 << col);
	}
	updateTX();
}

//----------------------------------------------------------------------------
void rk_keyboard(const hid_keyboard_report_t  *report, uint16_t len)
{

    //debug_print("0x%02X 0x%02X 0x%02X 0x%02X  0x%02X 0x%02X 0x%02X\r\n", report[0], report[2], report[3], report[4], report[5], report[6], report[7]);
    portC = 0xE0;
    rk_scancode_s(report->modifier); // запись в таблицу модификатора alt ctrl shift win

	tab_key[report->keycode[0]] = 1; // запись в таблицу 1 кода
    tab_key[report->keycode[1]] = 1; // запись в таблицу 2 кода
    tab_key[report->keycode[2]] = 1; // запись в таблицу 3 кода
    tab_key[report->keycode[3]] = 1; // запись в таблицу 4 кода
    tab_key[report->keycode[4]] = 1; // запись в таблицу 5 кода
    tab_key[report->keycode[5]] = 1; // запись в таблицу 6 кода

    for (uint8_t i = 1; i < 127; i++)
    {
        uint8_t d = (tab_key[i] << 1) | tab_key_old[i]; // 0b000000x0 |0b0000000y
        if (d == 2)
            rk_key(i, true); 		 // нажато сейчас
        if (d == 1)
            rk_key(i, false);        // клавиша отпущена  сейчас
        tab_key_old[i] = tab_key[i]; // копирование таблицы
        tab_key[i] = 0;              // стирание таблицы
    }
}

//----------------------------------------------------------

