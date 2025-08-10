#include <sam3x8e.h>
#include "Helix/mp3dec.h"
#include "Helix/mp3common.h"
#include "Helix/coder.h"
#include <string.h>

#define preBUF_SIZE8 1440		//uint8_t
#define iBUF_SIZE8 11520		//uint8_t
#define oBUF_SIZE16 1152*2		//uint_16_t
#define oNUM_BUF 4

MP3DecInfo mp3DecInfo;
FrameHeader fh;
SideInfo si;
ScaleFactorInfo sfi;
HuffmanInfo hi;
DequantInfo di;
IMDCTInfo mi;
SubbandInfo sbi;

uint16_t oData[oNUM_BUF][oBUF_SIZE16];
uint8_t FulliData[preBUF_SIZE8 + iBUF_SIZE8];
uint8_t* iData;
int bytesleft, bO, Samples;
unsigned char *read_ptr;

void (*Conv16to12)(int num);

void Conv16to12Stereo (int num)
{
	uint32_t i;
	for (i = 0; i < Samples/2; i++) {
		oData[num][2*i] =	(oData[num][2*i] ^ 32768)>>4;
		oData[num][2*i+1] = 0x1000 | ((oData[num][2*i+1] ^ 32768)>>4);
	}
}

void Conv16to12Mono (int num)
{
	uint32_t i;

	for (i = Samples/2 - 1; i > 0; i--) {
		oData[num][2*i] =	(oData[num][i] ^ 32768)>>4;
		oData[num][2*i+1] = 0x1000 | oData[num][2*i];
	}
	i = oData[num][0];
	oData[num][0] =	(i ^ 32768)>>4;
	oData[num][1] = 0x1000 | oData[num][0];
}

void MP3InitDecoder(void)
{
	mp3DecInfo.FrameHeaderPS =     &fh;
	mp3DecInfo.SideInfoPS =        &si;
	mp3DecInfo.ScaleFactorInfoPS = &sfi;
	mp3DecInfo.HuffmanInfoPS =     &hi;
	mp3DecInfo.DequantInfoPS =     &di;
	mp3DecInfo.IMDCTInfoPS =       &mi;
	mp3DecInfo.SubbandInfoPS =     &sbi;
}

void UART_Handler(void)
{

	REG_UART_RNPR = (uint32_t) iData;
	REG_UART_RNCR = iBUF_SIZE8;

}


void DACC_Handler(void)
{
	int err, bl;
	unsigned char *tmp_ptr;

	REG_PIOB_CODR = 1<<27;
	
	REG_DACC_TNPR = (uint32_t) oData[bO];
	REG_DACC_TNCR = Samples;
	bO = (bO + 1) & (oNUM_BUF-1);
	
	bl = bytesleft - REG_UART_RCR;
	if (bl < 0) bl += iBUF_SIZE8;
	
	if (bl < 2880) REG_UART_THR = 0x11;
	else if (bl > iBUF_SIZE8 - 2880) REG_UART_THR = 0x13;
		
	bl = bytesleft;
	tmp_ptr = read_ptr;

	err = MP3Decode (&mp3DecInfo, &read_ptr, &bytesleft, (short *)oData[bO], 0);
	if (err != 0) {
		if (bytesleft <= preBUF_SIZE8) {
			read_ptr = iData - bl;
			bytesleft = bl + iBUF_SIZE8;
			memcpy(read_ptr, tmp_ptr, bl);
			err = MP3Decode (&mp3DecInfo, &read_ptr, &bytesleft, (short *)oData[bO], 0);
		}
		if (err != 0) return;
	}
	Conv16to12(bO);

	//Reset WatchDog
	REG_WDT_CR = (0xA5u << 24) | WDT_CR_WDRSTT;
	
	REG_PIOB_SODR = 1<<27;
}



int main(void)
{
	MP3FrameInfo Inf;
	uint32_t i = 0;
	
	//SystemInit();
	//mySystemInit3(10);
	
	//For LED operations
	REG_PIOB_OER = 1<<27;	// Configure pin as output
	REG_PIOB_PER = 1<<27;
	
	//UART
	REG_PMC_PCER0 = 1<<ID_UART;
	// Disable interrupts on the pins
	REG_PIOA_IDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;
	//pio_set_peripheral A
	REG_PIOA_ABSR &= ~(PIO_PA8A_URXD | PIO_PA9A_UTXD);
	// Remove the pins from under the control of PIO
	REG_PIOA_PDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;
	// Configure mode
	REG_UART_MR = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;
	// Configure baudrate
	REG_UART_BRGR = SystemCoreClock/(16*500000);		//500 000 kbps
	//REG_UART_BRGR = SystemCoreClock/(16*250000);		//250 000 kbps
	// Configure PDC
	iData = FulliData + preBUF_SIZE8;	
	REG_UART_RPR = (uint32_t)iData;
	REG_UART_RCR = iBUF_SIZE8;
	REG_UART_RNPR = (uint32_t)iData;
	REG_UART_RNCR = iBUF_SIZE8;
	REG_UART_PTCR = PERIPH_PTCR_RXTEN;
	// Enable UART IRQ
	REG_UART_IER = UART_IER_ENDRX;
	NVIC_EnableIRQ(UART_IRQn);
	NVIC_SetPriority(UART_IRQn, 12);
	
	//TC0
	REG_PMC_PCER0 = 1 << 27;
	//  Set mode
	REG_TC0_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1 |		// Input clock = DACC Clock = MCK/2
	TC_CMR_WAVE |									// Waveform mode
	TC_CMR_WAVSEL_UP_RC |							//RC trigger
	TC_CMR_ACPA_CLEAR |								// RegA clear
	TC_CMR_ACPC_SET; 								// RegC set
	
	//Start to receive data from port
	REG_UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	
	REG_PIOC_OER = 1<<10;	// Configure pin as output
	REG_PIOC_PER = 1<<10;
	

	while(REG_UART_RCR > 32) {
		if(i) REG_PIOC_CODR = 1<<10; //Some sort of a glitch
		else REG_PIOC_SODR = 1<<10;
		i = 1-i;
	}
	REG_UART_THR = 0x13;
	
	MP3InitDecoder();
	read_ptr = iData;
	bytesleft = iBUF_SIZE8;
	if(!(*((uint16_t *)read_ptr))) {
		read_ptr += 2;
		bytesleft -= 2;
	}
	if((*((uint32_t *)read_ptr) & 0xFFFFFF) == 0x334449) {
		bytesleft -= (read_ptr[8]<<7) + read_ptr[9];
		read_ptr += (read_ptr[8]<<7) + read_ptr[9];
	}
	i = MP3FindSyncWord (read_ptr, bytesleft);
	read_ptr += i;
	bytesleft -= i;
	MP3Decode (&mp3DecInfo, &read_ptr, &bytesleft, (short *)oData[0], 0);
	MP3Decode (&mp3DecInfo, &read_ptr, &bytesleft, (short *)oData[1], 0);
	MP3Decode (&mp3DecInfo, &read_ptr, &bytesleft, (short *)oData[2], 0);
	bO = 2;
	
	MP3GetLastFrameInfo (&mp3DecInfo, &Inf);
	
	i = SystemCoreClock/(4*Inf.samprate);
	REG_TC0_RC0 = i;		// Period
	REG_TC0_RA0 = i/2;		// Duty
	Samples = Inf.outputSamps;
	if (Inf.nChans == 2) Conv16to12 = Conv16to12Stereo;
	else {
		Conv16to12 = Conv16to12Mono;
		Samples *= 2;
	}

	Conv16to12(0);
	Conv16to12(1);
	Conv16to12(2);

	//DACC
	REG_PMC_PCER1 = 1<<(ID_DACC-32);
	REG_DACC_MR = DACC_MR_TRGEN | DACC_MR_TRGSEL(1) | DACC_MR_TAG;
	// Configure PDC
	REG_DACC_TPR = (uint32_t) oData[0];
	REG_DACC_TCR = Samples;
	REG_DACC_TNPR = (uint32_t) oData[1];
	REG_DACC_TNCR = Samples;
	REG_DACC_PTCR = PERIPH_PTCR_TXTEN;
	REG_DACC_IER = DACC_IER_ENDTX;
	NVIC_EnableIRQ(DACC_IRQn);
	NVIC_SetPriority(DACC_IRQn, 11);
	
	//Satrt playing
	REG_DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1;
	REG_TC0_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;
	//REG_UART_THR = 0x11;
	
	//Program WatchDog ~1sec
	REG_WDT_MR = 255 | WDT_MR_WDRSTEN | WDT_MR_WDD(4095);
}
