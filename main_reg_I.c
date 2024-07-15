// ePWM ADC Interrupt and onboard LED + Carte verte blink
// L. BAGHLI 11/05/2024

// Included Files
#include "F28x_Project.h"
#include <math.h>

// attention : Si on change PWM TBPRD, il faut aussi changer Kdth
// 200MHz/2= 100 MHz max que peut supporter PWM module ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;
// 100MHz/2500/2 = 20 kHz
//#define EPWM_TIMER_TBPRD    2500      // 20 kHz
//#define FullPWM 2500
//#define HalfPWM 1250
#define EPWM_TIMER_TBPRD    5000      // 10 kHz
#define FullPWM 5000
#define HalfPWM 2500
#define EPWM_MAX_DB   0 //20  DB done by DRV8305

// Variables
int Count = 0;
Uint16 cmpr = HalfPWM;
Uint16 ADCIref=0, ADCImes=0;
double Iref=0, Imes=0, xe_I=0, alpha=0, e;
double Kp=0.05, Ki=0.0001;
Uint16 Ech = 0, Echelons = 0;
double IrefEch = 1.0;
Uint16 IrefEchT = 20;

// Function Prototypes
void Gpio_setup();
void initEPWM();
void InitMyADC();
interrupt void adca1_isr(void);
void RegulI();

void main(void)
{
  InitSysCtrl();
  Gpio_setup();
  initEPWM();
  ConfigureADC();
  InitMyADC();

  DINT;
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();

  EALLOW;
  PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
  EDIS;
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
  IER |= M_INT1; //Enable group 1 interrupts
  EINT;  // Enable Global interrupt INTM
  ERTM;  // Enable Global realtime interrupt DBGM

  // Step 6. IDLE loop. Just sit and loop forever (optional):
  for(;;);
}

// Regulation du courant
void RegulI()
{
  e = Iref - Imes;
  alpha = Kp*e + xe_I;
  if ( (alpha <0.5) && (alpha>-0.5) )	xe_I += Ki*e;
  // limiteur de tension
  if ( alpha > 0.5) alpha = 0.5;
  if ( alpha <-0.5) alpha =-0.5;
  cmpr = (alpha+0.5)*FullPWM;
}

interrupt void adca1_isr(void)
{
  GpioDataRegs.GPCSET.bit.GPIO67 = 1;
  Count++;
  GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

  ADCIref = AdcbResultRegs.ADCRESULT0;
  ADCImes = AdcaResultRegs.ADCRESULT0;
  if (Echelons)   IrefEch = ADCIref*0.00390625 -8;            // +/-8A
  else  Iref = ADCIref*0.00390625 -8;	        // +/-8A
  Imes = ADCImes*0.0107421875 - 33.333;	// calibre LTS25 à 8A

  if (Echelons)
      if (++Count >= IrefEchT)	{
        Count=0;
        Ech = ! Ech;
        if (Ech)	Iref = IrefEch;
        else	Iref =-IrefEch;
        if (Ech)	GpioDataRegs.GPASET.bit.GPIO22 = 1;
        else	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
      }
   RegulI();
 //cmpr = ADCIref;
  EPwm1Regs.CMPA.bit.CMPA = cmpr;
  EPwm1Regs.CMPB.bit.CMPB = cmpr;
  EPwm2Regs.CMPA.bit.CMPA = cmpr;
  EPwm2Regs.CMPB.bit.CMPB = cmpr;

  // Check if overflow has occurred
  if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
  {
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
  }
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
  GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
}

void Gpio_setup()
{
    // Enable PWM1-3 on GPIO0-GPIO5, I2C, BP, OUT1
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1;     // Disable pull-up on GPIO32
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;    // Configure GPIO32 as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;     // GPIO32 = input car emplacement du 69M ADCINA6 sur J1.02 P32

    GpioCtrlRegs.GPDPUD.bit.GPIO111 = 1;     // Disable pull-up on GPIO111
    GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;    // Configure GPIO111 as GPIO
    GpioCtrlRegs.GPDDIR.bit.GPIO111 = 0;     // GPIO111 = input car emplacement du 69M ADCINB6 sur J1.06 P11

    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;     // Disable pull-up on GPIO22
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    // Configure GPIO22 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // GPIO22 = output    OUT2
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   // Clear

    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 1;     // Disable pull-up on GPIO67
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;    // Configure GPIO67 as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;     // GPIO67 = output    OUT1
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;   // Clear

    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;     // Disable pull-up on GPIO19
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    // Configure GPIO19 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;     // GPIO19 = input    OUT2

    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;     // Disable pull-up on GPIO18
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;    // Configure GPIO18 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;     // GPIO18 = input    OUT2

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;       // Onboard LED Output
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPADAT.bit.GPIO31 = 1;       // Onboard LED Off
    GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

    // Enable I2C-A on J1.09 GPIO105 SCL_OLED et J1.10 GPIO104 SDA_OLED
    GpioCtrlRegs.GPDPUD.bit.GPIO105 = 0;   // Enable pullup on GPIO32
    GpioCtrlRegs.GPDMUX1.bit.GPIO105 = 1;  // GPIO105 = SDAA
    GpioCtrlRegs.GPDQSEL1.bit.GPIO105 = 3; // Asynch input
    GpioCtrlRegs.GPDPUD.bit.GPIO104 = 0;   // Enable pullup on GPIO33
    GpioCtrlRegs.GPDQSEL1.bit.GPIO104 = 3; // Asynch input
    GpioCtrlRegs.GPDMUX1.bit.GPIO104 = 1;  // GPIO104 = SCLA
    EDIS;
}

void initEPWM()
{
  EALLOW;
  // Configure the prescaler to the ePWM modules.  Max ePWM input clock is 100 MHz.
  ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;     // EPWMCLK divider from PLLSYSCLK.  0=/1, 1=/2
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
  EDIS;
  // setup the Time-Base Period Register (TBPRD)
  EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;     // Set timer period
  EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;     // Phase is 0
  EPwm1Regs.TBCTR = 0x0000;                // Clear counter
  EPwm1Regs.CMPA.bit.CMPA = cmpr;     // Set compare A value
  EPwm1Regs.CMPB.bit.CMPB = cmpr;               // Set Compare B value
  // Setup counter mode
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
  EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
  EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
  EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
  // Setup shadowing
  EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
  EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
  // Set actions
  EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;  // Set PWM1A on event A, up count
  EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;    // Clear PWM1A on event A, down count
  EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
  EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

//  EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//  EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // or DB_ACTV_HIC
//  EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//  EPwm1Regs.DBRED.bit.DBRED = EPWM_MAX_DB;
//  EPwm1Regs.DBFED.bit.DBFED = EPWM_MAX_DB;

  // setup the Time-Base Period Register (TBPRD)
  EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;     // Set timer period
  EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;     // Phase is 0
  EPwm2Regs.TBCTR = 0x0000;                // Clear counter
  EPwm2Regs.CMPA.bit.CMPA = cmpr;     // Set compare A value

  // Setup counter mode
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
  EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
  EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
  EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
  // Setup shadowing
  EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
  EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
  // Set actions
  EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
  EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
  EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

//  EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//  EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // or DB_ACTV_HIC
//  EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//  EPwm2Regs.DBRED.bit.DBRED = EPWM_MAX_DB;
//  EPwm2Regs.DBFED.bit.DBFED = EPWM_MAX_DB;


// Interruot only for PWM1
//  EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//  EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//  EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
  EDIS;
}

// ConfigureADC - Write ADC configurations and power up the ADC for both
// ADC A and ADC B
void ConfigureADC()
{
  EALLOW;
  //write configurations
  AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
  AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

  AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
  AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
  //Set pulse positions to late
  AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
  AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
  //power up the ADC
  AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
  AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
  //delay for 1ms to allow ADC time to power up
  DELAY_US(1000);
  EDIS;
}

// InitMyADC
void InitMyADC()
{
  Uint16 acqps;
  // Determine minimum acquisition window (in SYSCLKS) based on resolution
  acqps = 30; //75ns  for ADC_RESOLUTION_12BIT
  //Select the channels to convert and end of conversion flag
  EALLOW;
// ADCINA3 Imes
  AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC0 will convert pin A3
  AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
  AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
// ADCINB3 Iref
  AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC0 will convert pin B3
  AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
  AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C

  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

  // ePWM1 SOC à chaque Periode
  EPwm1Regs.ETSEL.bit.SOCAEN	= 1;			// Enable SOC on A group
  EPwm1Regs.ETSEL.bit.SOCASEL	= ET_CTR_ZERO;	// SOC on time-base counter equal to zero (TBCTR = zero)
  EPwm1Regs.ETPS.bit.SOCAPRD 	= ET_1ST;		// Generate pulse on 1st event
  EDIS;
}



