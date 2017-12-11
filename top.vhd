-------------------------------------------------------------------------------
-- Title         : Zynq RFBPM  Top
-------------------------------------------------------------------------------
-- File          : top.vhd
-- Author        : Joseph Mead  mead@bnl.gov
-- Created       : 5/5/2016
-------------------------------------------------------------------------------
-- Description:
-- Provides the logic for RFBPM DFE board
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
--  K.Ha kha@ bnl.gov
--
-- 
--
-- I2C connections for power management.
--
--
--
-- Data  PSM_SDA_FPGA connect to PS_MIO35_501_G21
-- Clock PSM_SCL_FPGA connect to PS_MIO34_501_K18
--
--
-- 07/14/17
--		TBT x, y signal changed
-- 
-- 07/18/17
--		added tbt/fa HS current address
--		added tbt/fa burst enable status
--
--	08/01/17
--		added tbt_irq and fa_irq
--
--  08/22/17
--      added 2nd gate and PT hi/lo processing blocks
--
--	08/24/17
--		dsp_ctrl2 added for gate2 control
--
--  08/25/17
--      gate2 tbt/fa a,b,c,d added to hp port
--      
--  09/19/17
--		ADC button A and D swapped.
--
--  09/19/17
--      XADC Wizard added
--
--	09/22/17
--		PT lo lookup table used Dual port RAM for programable sin/cos 
--	09/26/17
--		Added PT LO band pass filter enable/disable
--	09/28/17
--		Added PT LO SA phase values
--		Added Button SA phase readout
--
--	10/02/17
--		Added PT 502.897670180000e+006    33.5012749800000e+006	% default operation
--		Added button mag * ptgain
--
--  10/05/17
--      Added mix output I&Q, CHA and CHB
--      Added PT count monitoring from ADC waveform
--
--	10/06/17
--		tbt_wfm_a, b, c, d for PT LOW TBT waveform readadcs
--
--
--


library IEEE;
use IEEE.std_logic_1164.ALL;
--use IEEE.numeric_std.ALL;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
 

library work;
use work.bpm_package.ALL;
 

entity top is
generic(
    --FPGA_VERSION			: integer := 91917;
	--FPGA_VERSION			: integer := 92217;
	--FPGA_VERSION			: integer := 92617;	
	--FPGA_VERSION			: integer := 92817;
	--FPGA_VERSION			: integer := 100217;
	--FPGA_VERSION			: integer := 100517;
	FPGA_VERSION			: integer := 100617;
	
    VIVADO_VERSION          : integer := 20164;
    SIM_MODE				: integer := 0;
	SDI_ENABLE				: integer := 0           -- SDI communication module
    );
  port (
    ddr_addr                : inout std_logic_vector ( 14 downto 0 );
    ddr_ba                  : inout std_logic_vector ( 2 downto 0 );
    ddr_cas_n               : inout std_logic;
    ddr_ck_n                : inout std_logic;
    ddr_ck_p                : inout std_logic;
    ddr_cke                 : inout std_logic;
    ddr_cs_n                : inout std_logic;
    ddr_dm                  : inout std_logic_vector ( 3 downto 0 );
    ddr_dq                  : inout std_logic_vector ( 31 downto 0 );
    ddr_dqs_n               : inout std_logic_vector ( 3 downto 0 );
    ddr_dqs_p               : inout std_logic_vector ( 3 downto 0 );
    ddr_odt                 : inout std_logic;
    ddr_ras_n               : inout std_logic;
    ddr_reset_n             : inout std_logic;
    ddr_we_n                : inout std_logic;
    fixed_io_ddr_vrn        : inout std_logic;
    fixed_io_ddr_vrp        : inout std_logic;
    fixed_io_mio            : inout std_logic_vector ( 53 downto 0 );
    fixed_io_ps_clk         : inout std_logic;
    fixed_io_ps_porb        : inout std_logic;
    fixed_io_ps_srstb       : inout std_logic;
    
 
 	--adc inputs
    adc_clk_p               : in std_logic;
    adc_clk_n               : in std_logic;
 
    adc0_clk_p              : in std_logic;
    adc0_clk_n              : in std_logic;     
    adc0_data_p             : in std_logic_vector(15 downto 0);
    adc0_data_n             : in std_logic_vector(15 downto 0);
    adc0_ovr_p              : in std_logic;
    adc0_ovr_n              : in std_logic;                   

    adc1_clk_p              : in std_logic;
    adc1_clk_n              : in std_logic;
    adc1_data_p             : in std_logic_vector(15 downto 0);
    adc1_data_n             : in std_logic_vector(15 downto 0);             
    adc1_ovr_p              : in std_logic;
    adc1_ovr_n              : in std_logic;                 

    adc2_clk_p              : in std_logic;
    adc2_clk_n              : in std_logic;
    adc2_data_p             : in std_logic_vector(15 downto 0);
    adc2_data_n             : in std_logic_vector(15 downto 0);    
    adc2_ovr_p              : in std_logic;
    adc2_ovr_n              : in std_logic;                 

    adc3_clk_p              : in std_logic;
    adc3_clk_n              : in std_logic;
    adc3_data_p             : in std_logic_vector(15 downto 0);
    adc3_data_n             : in std_logic_vector(15 downto 0);    
    adc3_ovr_p              : in std_logic;
    adc3_ovr_n              : in std_logic;
           
    --adc control
--    adc_rand                : out std_logic;
--    adc_pga                 : out std_logic;
--    adc_shdn                : out std_logic;
--    adc_dith                : out std_logic;  
    
    -- adc clock synthesizer (AD9510)
    ad9510_sclk             : out std_logic;
    ad9510_sdata            : out std_logic;
    ad9510_lat              : out std_logic;
    ad9510_func             : out std_logic;
    ad9510_status           : in std_logic;
    ad9510_sdo              : in std_logic;
    
    -- pilot tone synthesizer
    lmx2541_spiclk          : out std_logic;
    lmx2541_spidata         : out std_logic;
    lmx2541_spistrobe       : out std_logic;
    lmx2541_lockdet         : in std_logic;
    lmx2541_spice           : out std_logic;    
    lmx2541_rfout_en        : out std_logic; 
    
    -- rf digital attenuator
    dsa0_clk                : out std_logic;
    dsa0_sdata              : out std_logic;
    dsa0_latch              : out std_logic;
    
    -- pilot tone digital attenuator
    dsa2_clk                : out std_logic;
    dsa2_sdata              : out std_logic;
    dsa2_latch              : out std_logic;
    
    -- tbt clock input
    tbt_clk_p               : in std_logic;
    tbt_clk_n               : in std_logic;
 
    -- power good signal 
    pwr_good                : in std_logic;

	-- GTX for SDI --
--	gtx_refclk_p			: in std_logic;
--	gtx_refclk_n			: in std_logic;
	
    -- Embedded Event Receiver
    gtx_evr_refclk_p        : in std_logic;
    gtx_evr_refclk_n        : in std_logic;
    gtx_evr_rx_p            : in std_logic;
    gtx_evr_rx_n            : in std_logic;
    evr_rcvd_clk_p          : out std_logic;
    evr_rcvd_clk_n          : out std_logic;
    
    --si5338 i2c
    si5338_i2c_sda          : inout std_logic;
    si5338_i2c_scl          : inout std_logic;

	
--	sfp0_i2c_scl			: inout std_logic;
--	sfp0_i2c_sda			: inout std_logic;
--	iic_sdi_sfp0_scl			: inout std_logic;
--	iic_sdi_sfp0_sda			: inout std_logic;
--	iic_sdi_sfp1_scl			: inout std_logic;
--	iic_sdi_sfp1_sda			: inout std_logic;
		
	--sfp I/O
	sfp_sclk					: inout std_logic_vector(5 downto 0);
	sfp_sdata					: inout std_logic_vector(5 downto 0);
		
	
    -- front panel I/O
    fp_out                  : out std_logic_vector(1 downto 0);
    fp_in                   : in  std_logic_vector(1 downto 0);
   
    --  LED's 
    dbg_leds                : out std_logic_vector(3 downto 0);
    sfp_leds                : out std_logic_vector(11 downto 0);
    fp_leds                 : out std_logic_vector(7 downto 0);
    
    -- Debug pins 
    --dbg                     : out std_logic_vector(25 downto 0);
    dbg                     : out std_logic_vector(20 downto 0);
	
	-- afe temperature sensors
	afetemp_scl					: out std_logic;
	afetemp_sda					: inout std_logic;

	-- dfe temperature sensors
	dfetemp_scl					: out std_logic;
	dfetemp_sda					: inout std_logic
		
    -- SYS clock input
--    sys_clk_p               : in std_logic;
--    sys_clk_n               : in std_logic
        
  );
end top;


architecture behv of top is

    COMPONENT trig_edge_gen
    PORT(
         clk : IN  std_logic;
         trig_in : IN  std_logic;
         trig_out : OUT  std_logic
        );
    END COMPONENT;
	
    COMPONENT trig_times
    PORT(
         reset : IN  std_logic;
         evr_clk : IN  std_logic;
         adc_clk : IN  std_logic;
         trig_in : IN  std_logic;
         evr_ts_in : IN  std_logic_vector(63 downto 0);
         trig_ts : OUT  std_logic_vector(63 downto 0)
        );
    END COMPONENT; 
	
	 component trig_sel is
		port (
			adc_clk         : in  std_logic;  
			reset           : in  std_logic;
			soft_trig       : in  std_logic;
			evr_trig        : in  std_logic;
			fp_trig         : in  std_logic;
			evr_soft_trig   : in  std_logic;
			trig_sel        : in  std_logic_vector(1 downto 0);
			trig_dly_reg    : in  std_logic_vector(31 downto 0);
			npi_trig        : out std_logic
		  );    
	 end component;
 
    COMPONENT tbt_hp_ddr
    PORT(
		adc_clk : IN  std_logic;
		rst : IN  std_logic;
		tbt_trig : IN  std_logic;
		endian_en : IN  std_logic;
		but_va : IN  std_logic_vector(31 downto 0);
		but_vb : IN  std_logic_vector(31 downto 0);
		but_vc : IN  std_logic_vector(31 downto 0);
		but_vd : IN  std_logic_vector(31 downto 0);
		but_xpos : IN  std_logic_vector(31 downto 0);
		but_ypos : IN  std_logic_vector(31 downto 0);
		but_sum : IN  std_logic_vector(31 downto 0);

		but_va2 : IN  std_logic_vector(31 downto 0);
		but_vb2 : IN  std_logic_vector(31 downto 0);
		but_vc2 : IN  std_logic_vector(31 downto 0);
		but_vd2 : IN  std_logic_vector(31 downto 0);

		ddr_data : OUT  std_logic_vector(31 downto 0);
		ddr_data_valid : OUT  std_logic
        );
    END COMPONENT;
	

    COMPONENT data2ddr_tf
    PORT(
         sys_clk : IN  std_logic;
         adc_clk : IN  std_logic;
         reset : IN  std_logic;
         trig : IN  std_logic;
         burst_enb : IN  std_logic;
         burst_len : IN  std_logic_vector(31 downto 0);
         testdata_en : IN  std_logic;
         fifo_din_32bit : IN  std_logic_vector(31 downto 0);
         fifo_din_valid : IN  std_logic;
         hs_fifo_rdcnt : OUT  std_logic_vector(8 downto 0);
         hs_fifo_rddata : OUT  std_logic_vector(63 downto 0);
         hs_fifo_empty : OUT  std_logic;
         hs_fifo_rden : IN  std_logic;
         hs_fifo_rst : IN  std_logic;
         hs_tx_enb : OUT  std_logic;
         hs_tx_active : OUT  std_logic;
         dbg_strobe_lat : OUT  std_logic
        );
    END COMPONENT;

	
	-- Verilog source
	component bpm_countdatagen
	port  (
			sysClk : 			in std_logic;
			LtableRamInData : 	in std_logic_vector(31 downto 0);
			phaseCnt : 			in std_logic_vector(9 downto 0);
			
			Reset         : in std_logic; 
			NCO_reset     : in std_logic;   
			sdi_clk       : in std_logic;  
			Trigger       : in std_logic;
			testMode	  : in std_logic_vector(3 downto 0);
			maxCount      : in std_logic_vector(31 downto 0);
			fa_pos_x      : in std_logic_vector(31 downto 0);
			fa_pos_y      : in std_logic_vector(31 downto 0);
			DataSel		  : in 	std_logic; 		
			wfmKx				:	in std_logic_vector(31 downto 0);
			wfmKy				:	in std_logic_vector(31 downto 0);
			wfmPhase_inc		:	in std_logic_vector(31 downto 0);
			-- OUTPUT		
			LocalDataValid    	: out std_logic;
			LocalCountData    : out std_logic_vector(31 downto 0);
			LocalBpmPosData   : out std_logic_vector(31 downto 0);  
			RampRst           : out std_logic;
			fa_evr_trig       : out std_logic;
			TrigOut				: out std_logic;
			glitch_out        : out std_logic;
			test_bit			: out std_logic;
			sdi_fa_pos_x    : out std_logic_vector(31 downto 0);
			sdi_fa_pos_y   : out std_logic_vector(31 downto 0)		
	   );
	end component;
	
	
	COMPONENT ila_tbt_ddr
	PORT (
		clk : IN STD_LOGIC;
		probe0 : IN STD_LOGIC_VECTOR(31 DOWNTO 0); 
		probe1 : IN STD_LOGIC_VECTOR(8 DOWNTO 0); 
		probe2 : IN STD_LOGIC_VECTOR(63 DOWNTO 0); 
		probe3 : IN STD_LOGIC_VECTOR(31 DOWNTO 0); 
		probe4 : IN STD_LOGIC_VECTOR(0 DOWNTO 0); 
		probe5 : IN STD_LOGIC_VECTOR(0 DOWNTO 0); 
		probe6 : IN STD_LOGIC_VECTOR(0 DOWNTO 0); 
		probe7 : IN STD_LOGIC_VECTOR(0 DOWNTO 0); 
		probe8 : IN STD_LOGIC_VECTOR(0 DOWNTO 0); 
		probe9 : IN STD_LOGIC_VECTOR(0 DOWNTO 0); 
		probe10 : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
		probe11 : IN STD_LOGIC_VECTOR(0 DOWNTO 0)
	);
	END COMPONENT  ;

	
	COMPONENT ila_adc_raw
	PORT (
		clk : IN STD_LOGIC;
		probe0 : IN STD_LOGIC_VECTOR(15 DOWNTO 0); 
		probe1 : IN STD_LOGIC_VECTOR(15 DOWNTO 0); 
		probe2 : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		probe3 : IN STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
	END COMPONENT  ;


	
------------------------------------------------------------	
	
   signal clk200mhz         : std_logic; 
   signal sys_clk           : std_logic; 
   signal sys_rst           : std_logic;
   signal sys_rstb          : std_logic_vector(0 downto 0);

   signal iobus_addr        : std_logic_vector(15 downto 0);
   signal iobus_cs          : std_logic;
   signal iobus_rnw         : std_logic;
   signal iobus_rddata      : std_logic_vector(31 downto 0);
   signal iobus_wrdata      : std_logic_vector(31 downto 0);
   
   signal iobus_leds        : std_logic_vector(31 downto 0);
        
   signal sysgen_version    : std_logic_vector(31 downto 0);  
   
   signal soft_trig         : std_logic;   

   signal hs_cntrl          : std_logic_vector(7 downto 0);
   signal hs_status         : std_logic_vector(3 downto 0);
   signal hs_burstlen       : std_logic_vector(31 downto 0);
   signal hs_ddrbaseaddr    : std_logic_vector(31 downto 0);
   signal hs_ddrcuraddr     : std_logic_vector(31 downto 0);  
   signal hs_ddrbuflen      : std_logic_vector(31 downto 0); 
   signal hs_throttle       : std_logic_vector(31 downto 0);   
    
   signal hs_fifo_rdcnt     : std_logic_vector (11 downto 0);
   signal hs_fifo_rddata    : std_logic_vector (63 downto 0);
   signal hs_fifo_empty     : std_logic;
   signal hs_fifo_rden      : std_logic;
   signal hs_tx_enb         : std_logic;
   signal hs_tx_active      : std_logic;
   signal hs_axi_tx_active  : std_logic;
  
   signal tenhz_irq         : std_logic;
   signal other_irq         : std_logic;
   signal fa_irq            : std_logic;
   signal tbt_irq           : std_logic;
   
   
   signal lmx2541_we        : std_logic; 
   signal lmx2541_data      : std_logic_vector(31 downto 0);
   signal lmx2541_rfenb     : std_logic;
   

   signal dsa0_we			: std_logic;
   signal dsa2_we			: std_logic;
   signal dsa_data		    : std_logic_vector(7 downto 0);
   signal dsa0_debug		: std_logic_vector(8 downto 0);
   signal dsa0_clk_i		: std_logic;
   signal dsa0_sdata_i		: std_logic;
   signal dsa0_latch_i		: std_logic;
   signal dsa2_debug	    : std_logic_vector(8 downto 0);	   
   
   
 
   --ad9510
   signal ad9510_we         : std_logic;
   signal ad9510_data       : std_logic_vector(31 downto 0);    
   signal ad9510_debug      : std_logic_vector(8 downto 0);
   signal ad9510_sclk_i     : std_logic;
   signal ad9510_sdata_i    : std_logic;
   signal ad9510_lat_i      : std_logic;
   
   signal adc_clk_stopped   : std_logic;
   signal adc_clk_locked    : std_logic;
   signal adc_fifo_wren     : std_logic;
   signal adc_dbg_sel       : std_logic_vector(31 downto 0);    
   signal dds_adcsim_din    : std_logic_vector(31 downto 0);
   signal dds_adcsim_we     : std_logic_vector(2 downto 0);          
         
   signal adc_clk           : std_logic;
   signal adc_sat           : std_logic_vector(3 downto 0);
   
   signal adca_raw          : std_logic_vector(15 downto 0);
   signal adcb_raw          : std_logic_vector(15 downto 0);
   signal adcc_raw          : std_logic_vector(15 downto 0);
   signal adcd_raw          : std_logic_vector(15 downto 0);
   --
   signal adca_data         : std_logic_vector(15 downto 0);
   signal adcb_data         : std_logic_vector(15 downto 0);
   signal adcc_data         : std_logic_vector(15 downto 0);
   signal adcd_data         : std_logic_vector(15 downto 0); 
	--   
   signal adca_filt         : std_logic_vector(15 downto 0);
   signal adcb_filt         : std_logic_vector(15 downto 0);
   signal adcc_filt         : std_logic_vector(15 downto 0);
   signal adcd_filt         : std_logic_vector(15 downto 0);      
    
   signal adc_chb_but2      : std_logic_vector(15 downto 0);
   signal adc_chc_but2      : std_logic_vector(15 downto 0);
   signal adc_cha_but2      : std_logic_vector(15 downto 0);
   signal adc_chd_but2      : std_logic_vector(15 downto 0); 	
		
   signal pt_va_lo_filt     : std_logic_vector(15 downto 0);
   signal pt_vb_lo_filt     : std_logic_vector(15 downto 0);
   signal pt_vc_lo_filt     : std_logic_vector(15 downto 0);
   signal pt_vd_lo_filt     : std_logic_vector(15 downto 0); 
   
   signal pt_va_hi     : std_logic_vector(15 downto 0);
   signal pt_vb_hi     : std_logic_vector(15 downto 0);
   signal pt_vc_hi     : std_logic_vector(15 downto 0);
   signal pt_vd_hi     : std_logic_vector(15 downto 0); 
   

			
   signal adc_fifo_empty    : std_logic_vector(3 downto 0);
   
   
   signal tbt_extclk        : std_logic;
   
   signal inttrig_enb      : std_logic_vector(3 downto 0);
   signal tbt_gate         : std_logic;
   signal tbt_trig         : std_logic_vector(0 downto 0);
   signal pt_trig          : std_logic;
   signal fa_trig          : std_logic_vector(0 downto 0);
   signal sa_trig          : std_logic_vector(0 downto 0);
   signal dma_trig         : std_logic;
   
   signal tbt_sum          : std_logic_vector(31 downto 0);
   signal tbt_cha          : std_logic_vector(31 downto 0);
   signal tbt_chb          : std_logic_vector(31 downto 0);      
   signal tbt_chc          : std_logic_vector(31 downto 0);   
   signal tbt_chd          : std_logic_vector(31 downto 0);      
   
   signal but_coslkup      : std_logic_vector(15 downto 0); 
   signal but_sinlkup      : std_logic_vector(15 downto 0);
   
   signal sa_trig_stretch  : std_logic;
   signal dma_trig_stretch : std_logic; 
   
   signal dsp_filt_bypass   : std_logic_vector(3 downto 0);

   signal kx                : std_logic_vector(31 downto 0); 
   signal ky                : std_logic_vector(31 downto 0);
   signal cha_gain          : std_logic_vector(15 downto 0);
   signal chb_gain          : std_logic_vector(15 downto 0);
   signal chc_gain          : std_logic_vector(15 downto 0);
   signal chd_gain          : std_logic_vector(15 downto 0);
   signal xpos_offset       : std_logic_vector(31 downto 0);
   signal ypos_offset       : std_logic_vector(31 downto 0);
   signal xpos_offset2      : std_logic_vector(31 downto 0);
   signal ypos_offset2      : std_logic_vector(31 downto 0);
   
   signal evr_ts            : std_logic_vector(63 downto 0); 
   signal evr_ts_trig       : std_logic_vector(63 downto 0);
   signal evr_mapping_ram   : std_logic_vector(31 downto 0);
   signal evr_mapping_ram_we: std_logic;
   signal evr_trig_out      : std_logic_vector(7 downto 0);
   signal evr_clk           : std_logic;
   signal evr_rcvd_clk      : std_logic;
   
   signal evr_tbt_trig      : std_logic;
   signal evr_fa_trig       : std_logic;
   signal evr_sa_trig       : std_logic;
   signal evr_gps_trig      : std_logic;
   signal evr_dma_trig      : std_logic;
   signal evr_dbg           : std_logic_vector(19 downto 0);
   
	signal sys_status        : std_logic_vector(31 downto 0);
   
	signal adc_trig_count    : std_logic_vector(31 downto 0);

	signal sa_count          : std_logic_vector(31 downto 0);
	signal sa_cha            : std_logic_vector(31 downto 0);     
	signal sa_chb            : std_logic_vector(31 downto 0);     
	signal sa_chc            : std_logic_vector(31 downto 0);                     
	signal sa_chd            : std_logic_vector(31 downto 0);             
	signal sa_xpos           : std_logic_vector(31 downto 0);                   
	signal sa_ypos           : std_logic_vector(31 downto 0);                   
	signal sa_sum            : std_logic_vector(31 downto 0);
	signal but_q_sa          : std_logic_vector(31 downto 0);   

	signal i2c1_scl_i        : std_logic;
	signal i2c1_scl_t        : std_logic;
	signal i2c1_scl_o        : std_logic;
	signal i2c1_sda_i        : std_logic;
	signal i2c1_sda_t        : std_logic;
	signal i2c1_sda_o        : std_logic;     
   
	signal	iic_rtl_scl_i : std_logic;	--=> iic_rtl_scl_i;
	signal	iic_rtl_scl_o : std_logic;	--=> iic_rtl_scl_o;
	signal	iic_rtl_scl_t : std_logic;	--=> iic_rtl_scl_t;
	signal	iic_rtl_sda_i : std_logic;	--=> iic_rtl_sda_i;
	signal	iic_rtl_sda_o : std_logic;	--=> iic_rtl_sda_o;
	signal	iic_rtl_sda_t : std_logic;	--=> iic_rtl_sda_o;   
	signal  iic_sel : STD_LOGIC_VECTOR ( 0 to 0 );


	signal  iic_sdi_sfp0_scl_i : std_logic;	--in STD_LOGIC;
	signal  iic_sdi_sfp0_scl_o : std_logic;	--out STD_LOGIC;
	signal  iic_sdi_sfp0_scl_t : std_logic;	--out STD_LOGIC;
	signal  iic_sdi_sfp0_sda_i : std_logic;	--in STD_LOGIC;
	signal  iic_sdi_sfp0_sda_o : STD_LOGIC;
	signal  iic_sdi_sfp0_sda_t : STD_LOGIC;
	--
	signal  iic_sdi_sfp1_scl_i : std_logic; --STD_LOGIC;
	signal  iic_sdi_sfp1_scl_o : std_logic; --STD_LOGIC;
	signal  iic_sdi_sfp1_scl_t : std_logic; --STD_LOGIC;
	signal  iic_sdi_sfp1_sda_i : std_logic; --STD_LOGIC;
	signal  iic_sdi_sfp1_sda_o : std_logic; --STD_LOGIC;
	signal  iic_sdi_sfp1_sda_t : std_logic; --STD_LOGIC;
		
   
	-- EVR signals
--	signal sysClk50M        : std_logic;
--	signal evrTrig0_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig1_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig2_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig3_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig4_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig5_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig6_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig7_dly : std_logic_vector(31 downto 0)  := X"00000002";
--	signal evrTrig0_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig1_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig2_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig3_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig4_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig5_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig6_width : std_logic_vector(31 downto 0) := X"00000100";
--	signal evrTrig7_width : std_logic_vector(31 downto 0) := X"00000100";
	signal evrTrigOut : std_logic_vector(7 downto 0);		

	-- temperature sensor
    signal dfe_temp0	  		   : std_logic_vector(15 downto 0);
    signal dfe_temp1	  		   : std_logic_vector(15 downto 0);
    signal dfe_temp2		   	   : std_logic_vector(15 downto 0);
    signal dfe_temp3		  	   : std_logic_vector(15 downto 0);	
    signal dfe_temp_debug 	       : std_logic_vector(7 downto 0);	

    signal afe_temp0	  		   : std_logic_vector(15 downto 0);
    signal afe_temp1	  		   : std_logic_vector(15 downto 0);

    signal afe_temp_debug 	       : std_logic_vector(7 downto 0);
	

	--
    signal npi_trig_dly         : std_logic_vector(31 downto 0);
    signal npi_trig_sel         : std_logic_vector(1 downto 0);		
	
	signal sdi_ctrl             : std_logic_vector(31 downto 0);
	
	signal sdi_device_addr      : std_logic_vector(31 downto 0);
	signal sdi_packet_len       : std_logic_vector(11 downto 0);
	signal evr_width_trig0      : std_logic_vector(31 downto 0);	

	
	signal adc_data_endian_en	: std_logic; 
	signal adc_ddr_data_sel		: std_logic_vector(1 downto 0);
	signal locked               : std_logic; 
	--
    signal S00_AXIS_tdata       :  STD_LOGIC_VECTOR ( 63 downto 0 );
    signal S00_AXIS_tready      :  STD_LOGIC;
    signal S00_AXIS_tvalid      :  STD_LOGIC;	


	------------------------------------------
	signal evr_single_trig_out          : std_logic;
	signal soft_trig_out	     : std_logic;
	signal ddr_trig_start        : std_logic;
	signal tbt_hs_ddr_baseaddr    : std_logic_vector(31 downto 0);	-- from arm	DDR BASE address
	signal tbt_hs_ddr_buflen     : std_logic_vector(31 downto 0);	-- from arm Burst Length
	signal tbt_hs_burst_len      : std_logic_vector(31 downto 0);
	
	signal fa_hs_ddr_baseaddr    : std_logic_vector(31 downto 0);
	signal fa_hs_ddr_buflen      : std_logic_vector(31 downto 0);	
	signal fa_hs_burst_len       : std_logic_vector(31 downto 0);	
	
	signal tbt_hs_ddr_curaddr    : std_logic_vector(31 downto 0);
	signal fa_hs_ddr_curaddr     : std_logic_vector(31 downto 0);
	
    signal tbt_ddr_data_in       : std_logic_vector(31 downto 0);
    signal tbt_ddr_data_valid    : std_logic;
    signal fa_ddr_data_in        : std_logic_vector(31 downto 0);
    signal fa_ddr_data_valid     : std_logic;
	
    signal tbt_fa_ddr_data_in    : std_logic_vector(31 downto 0);
    signal tbt_fa_ddr_data_valid : std_logic;
	signal tbt_fa_hs_burst_len   : std_logic_vector(31 downto 0);
	
	
	-- tbt
	signal tbt_hs_fifo_rdcnt     : std_logic_vector (8 downto 0);
	signal tbt_hs_fifo_rddata    : std_logic_vector (63 downto 0);	
	signal tbt_hs_fifo_rden      : std_logic;
	signal tbt_hs_tx_enb         : std_logic;
	signal tbt_hs_tx_active      : std_logic;		
	signal tbt_hs_fifo_empty     : std_logic;
	signal tbt_hs_axi_tx_active  : std_logic;
	signal tbt_hs_dbg_strobe_lat  : std_logic;
	
   
	-- fa
	signal fa_hs_fifo_rdcnt     : std_logic_vector (8 downto 0);
	signal fa_hs_fifo_rddata    : std_logic_vector (63 downto 0);	
	signal fa_hs_fifo_rden      : std_logic;
	signal fa_hs_tx_enb         : std_logic;
	signal fa_hs_tx_active      : std_logic;	
	signal fa_hs_fifo_empty     : std_logic;
	signal fa_hs_axi_tx_active  : std_logic;
	-------------------------------------------------------------
	-- tbt signals
	signal but_pha_tbt		    : std_logic_vector(31 downto 0);	
    signal but_phb_tbt		    : std_logic_vector(31 downto 0);	
    signal but_phc_tbt		    : std_logic_vector(31 downto 0);	
    signal but_phd_tbt		    : std_logic_vector(31 downto 0);			
	signal but_xpos_tbt 	    : std_logic_vector(31 downto 0);	
	signal but_ypos_tbt     	: std_logic_vector(31 downto 0);
	signal but_xpos_nm_tbt 	    : std_logic_vector(31 downto 0);	
	signal but_ypos_nm_tbt     	: std_logic_vector(31 downto 0);
	
	
	signal but_va_fa		    : std_logic_vector(31 downto 0);	
    signal but_vb_fa		    : std_logic_vector(31 downto 0);	
    signal but_vc_fa		    : std_logic_vector(31 downto 0);	
    signal but_vd_fa		    : std_logic_vector(31 downto 0);			
	signal but_xpos_fa 	    	: std_logic_vector(31 downto 0);	
	signal but_ypos_fa     		: std_logic_vector(31 downto 0);		
	signal but_sum_fa 	    	: std_logic_vector(31 downto 0);	
	signal but_q_fa     		: std_logic_vector(31 downto 0);		

	
    	
    signal but_va_sa2		    : std_logic_vector(31 downto 0);	
    signal but_vb_sa2		    : std_logic_vector(31 downto 0);			
	signal but_vc_sa2 	    	: std_logic_vector(31 downto 0);	
	signal but_vd_sa2     		: std_logic_vector(31 downto 0);	
	signal but_sum_sa2		    : std_logic_vector(31 downto 0);	
	signal but_xpos_sa2 	    : std_logic_vector(31 downto 0);	
	signal but_ypos_sa2     	: std_logic_vector(31 downto 0);		
		
			
		
	signal dbg_tbt_hs_fifo_rden      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_tbt_hs_axi_tx_active      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_tbt_hs_tx_enb      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_tbt_hs_tx_active      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_tbt_hs_fifo_empty      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_tbt_ddr_data_valid      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_ddr_trig_start      : STD_LOGIC_VECTOR(0 DOWNTO 0);
	signal dbg_tbt_hs_dbg_strobe_lat      : STD_LOGIC_VECTOR(0 DOWNTO 0);

	


	signal test_reg0     		: std_logic_vector(31 downto 0);
	signal test_reg1     		: std_logic_vector(31 downto 0);
	signal pT_LoDpramData   	: std_logic_vector(31 downto 0);
	signal pT_LoDpramAddr   	: std_logic_vector(31 downto 0);	

		
	signal pt_va_sa_hi      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_vb_sa_hi      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_vc_sa_hi      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_vd_sa_hi      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_va_sa_lo      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_vb_sa_lo      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_vc_sa_lo      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal pt_vd_sa_lo      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	-- PT LO phase
	signal but_ptLoPhase_sA	: std_logic_vector(31 downto 0);	
	signal but_ptLoPhase_sB	: std_logic_vector(31 downto 0);	
	signal but_ptLoPhase_sC	: std_logic_vector(31 downto 0);	
	signal but_ptLoPhase_sD	: std_logic_vector(31 downto 0);
		
		
	signal but_pha_sa       : std_logic_vector(31 downto 0);
	signal but_pha_sb       : std_logic_vector(31 downto 0);
	signal but_pha_sc       : std_logic_vector(31 downto 0);
	signal but_pha_sd       : std_logic_vector(31 downto 0);
	signal but_phase_Q_sa   : std_logic_vector(31 downto 0);
	signal but_phase_I_sa   : std_logic_vector(31 downto 0);

			
	-- 2nd Gate signals
	
	signal tbt_gate2_ctrl      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal tbt_gate1:  std_logic_vector(0 downto 0); 
	signal tbt_gate2:  std_logic_vector(0 downto 0); 
	signal tbt_trig2:  std_logic; 
	signal tbt_trig_src : std_logic_vector(0 downto 0);
	
	
	signal but_sum_tbt2     : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_va_tbt2      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_vb_tbt2      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_vc_tbt2      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_vd_tbt2      : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_xpos_nm_tbt2 : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_ypos_nm_tbt2 : STD_LOGIC_VECTOR(31 DOWNTO 0);

	--
	signal but_va_fa2       : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_vb_fa2       : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_vc_fa2       : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_vd_fa2       : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_xpos_fa2     : STD_LOGIC_VECTOR(31 DOWNTO 0);
	signal but_ypos_fa2     : STD_LOGIC_VECTOR(31 DOWNTO 0);

	signal pt_coslkup_lo : STD_LOGIC_VECTOR(15 DOWNTO 0);
	signal pt_sinlkup_lo : STD_LOGIC_VECTOR(15 DOWNTO 0);			
	signal pt_coslkup    : STD_LOGIC_VECTOR(15 DOWNTO 0);
	signal pt_sinlkup    : STD_LOGIC_VECTOR(15 DOWNTO 0);	

	
	
    signal pt_ram_data   :  STD_LOGIC_VECTOR ( 31 downto 0 );
    signal pt_ram_addr   :  STD_LOGIC_VECTOR ( 9 downto 0 );
    signal pt_ram_sin_we :  STD_LOGIC_VECTOR ( 0 to 0 );
    signal pt_ram_cos_we  :  STD_LOGIC_VECTOR ( 0 to 0 );

	
	signal pt_tbt_gate_ctrl     :  STD_LOGIC_VECTOR(31 DOWNTO 0);
    signal pt_lo_filt_bypass 	:  std_logic_vector( 1-1 downto 0 );
    signal pt_lo_gate_en 		:  std_logic_vector( 1-1 downto 0 );	
    signal pt_sa_gain_a 		:  std_logic_vector( 24-1 downto 0 );
    signal pt_sa_gain_b 		:  std_logic_vector( 24-1 downto 0 );
    signal pt_sa_gain_c 		:  std_logic_vector( 24-1 downto 0 );
    signal pt_sa_gain_d 		:  std_logic_vector( 24-1 downto 0 );

	signal pt_lo_nco_count      :  std_logic_vector( 9 downto 0 );
	signal pt_lo_nco_count16    :  std_logic_vector( 15 downto 0 );
	
    signal pt_lo_imag_cha 		:  std_logic_vector( 16-1 downto 0 );
    signal pt_lo_qmag_cha 		:  std_logic_vector( 16-1 downto 0 );
    signal pt_lo_imag_chb 		:  std_logic_vector( 16-1 downto 0 );
    signal pt_lo_qmag_chb 		:  std_logic_vector( 16-1 downto 0 );
	

    signal pt_va_tbt_lo :  std_logic_vector( 32-1 downto 0 );
    signal pt_vb_tbt_lo :  std_logic_vector( 32-1 downto 0 );
    signal pt_vc_tbt_lo :  std_logic_vector( 32-1 downto 0 );
    signal pt_vd_tbt_lo :  std_logic_vector( 32-1 downto 0 );	
	
    signal tbt_wfm_a    :  std_logic_vector( 32-1 downto 0 );
    signal tbt_wfm_b    :  std_logic_vector( 32-1 downto 0 );
    signal tbt_wfm_c    :  std_logic_vector( 32-1 downto 0 );
    signal tbt_wfm_d    :  std_logic_vector( 32-1 downto 0 );
	
	
	
	-----////////////// S D I  C O M M ///////////////-----
	signal sdi_evr_fa_trig	: std_logic;
	signal sdi_glitch_out	: std_logic; 
	signal sdi_test_bit 	: std_logic; 	
	signal sdi_fa_pos_x		: std_logic_vector(31 downto 0);
	signal sdi_fa_pos_y		: std_logic_vector(31 downto 0);	
	signal sdi_fa_xpos      : std_logic_vector(31 downto 0);
	signal sdi_fa_ypos      : std_logic_vector(31 downto 0); 
	
	signal Sdi_WfmFreq	    : std_logic_vector(31 downto 0);
	signal Sdi_Wfm_Kx		: std_logic_vector(31 downto 0);
	signal Sdi_Wfm_Ky		: std_logic_vector(31 downto 0);

	signal tbt_gate_dly		    : std_logic_vector(31 downto 0);
	signal tbt_gate_width	    : std_logic_vector(15 downto 0);

	signal sfp_i2c_trig            : std_logic;
    signal sfp_i2c_addr            : std_logic_vector(8 downto 0);
    signal sfp0_i2c_data           : std_logic_vector(15 downto 0);
    signal sfp1_i2c_data           : std_logic_vector(15 downto 0);    
    signal sfp2_i2c_data           : std_logic_vector(15 downto 0);    
    signal sfp3_i2c_data           : std_logic_vector(15 downto 0);
    signal sfp4_i2c_data           : std_logic_vector(15 downto 0);
    signal sfp5_i2c_data           : std_logic_vector(15 downto 0);
	signal sfp_i2c_trig_src		   : std_logic;
	
	--Inputs
	signal fclk_clk2_25MHz : std_logic;
	--signal sysClk : std_logic := '0';
	--signal SdiReset : std_logic := '0';
	signal UserSdiReset : std_logic;
	--signal gtx_refclk_n : std_logic;
	--signal gtx_refclk_p : std_logic;
	signal RXN_IN : std_logic_vector(1 downto 0) := (others => '0');
	signal RXP_IN : std_logic_vector(1 downto 0) := (others => '0');
	signal SdiTrig_i : std_logic;
	signal SdiUsrNpiTrigger : std_logic;
	signal SdiUsrRstTrig : std_logic;
	signal DeviceAddress : std_logic_vector(15 downto 0) := (others => '0');
	signal PacketLength : std_logic_vector(11 downto 0) := (others => '0');
	--signal CWLocalDataClock : std_logic := '0';
	
	signal sdi_LocalDataValid : std_logic;
	signal sdi_LocalData : std_logic_vector(31 downto 0) := (others => '0');
	signal bpm_SimLocalData : std_logic_vector(31 downto 0) := (others => '0');
	signal CWLocalData : std_logic_vector(31 downto 0) := (others => '0');
	signal CWLocalDataValid : std_logic := '0';
	--signal CCWLocalData : std_logic_vector(31 downto 0) := (others => '0');
	--signal CCWLocalDataValid : std_logic := '0';
	
	
	signal DpramOutMode : std_logic := '0';
	signal LinkOutDir : std_logic_vector(1 downto 0) := (others => '0');
	signal DpRamTrigSrc : std_logic := '0';
	signal MaskContrl : std_logic_vector(11 downto 0) := (others => '0');
	--signal MasterPacketAddress : std_logic_vector(15 downto 0) := (others => '0');
	signal axiBusClk : std_logic := '0';
	signal axi_bus_en : std_logic := '0';
	signal axi_bus_addr : std_logic_vector(9 downto 0) := (others => '0');

	
	--Outputs
	signal TXN_OUT : std_logic_vector(1 downto 0);
	signal TXP_OUT : std_logic_vector(1 downto 0);
	signal StartCalc : std_logic;
	signal DPRAM_ReadStart : std_logic;
	signal DPRAM_Valid : std_logic;
	signal DPRAM_WR_OUT : std_logic;
	signal axi_bus_data : std_logic_vector(31 downto 0);
	signal NipStrobe : std_logic;
	signal NpiRst_Out : std_logic;
	signal TxClock125MHz : std_logic;
	signal LinkHead : std_logic_vector(31 downto 0);
	signal LinkData : std_logic_vector(31 downto 0);
	signal LinkDataValid : std_logic;
	signal LinkStartOfPacket : std_logic_vector(1 downto 0);
	signal CWLocalModeState : std_logic;
	signal CCWLocalModeState : std_logic;
	signal CWRemoteModeState : std_logic;
	signal CCWRemoteModeState : std_logic;
	signal sfp_link : std_logic_vector(7 downto 0);
	signal CW_LedRxFifoValid : std_logic;
	signal AllLinkValidStatus : std_logic;
	signal DirLED : std_logic;
	signal CCW_LinkDataValid : std_logic;
	signal BiDirLinkDataValid : std_logic;
	signal RemotePacketDroppedStatus : std_logic_vector(3 downto 0);
	signal CWRemoteModeDone_o : std_logic;
	signal CCWRemoteModeDone_o : std_logic;
	signal MyLocalLoopbackDataValid_o : std_logic_vector(3 downto 0);
	signal CW_RcvPacketMaskStatus : std_logic;
	signal CCW_RcvPacketMaskStatus : std_logic;
	signal ReceivedRemotePacketHead_s : std_logic_vector(3 downto 0);
	signal CWSingleSdiDebugOut : std_logic_vector(15 downto 0);
	signal CCWSingleSdiDebugOut : std_logic_vector(15 downto 0);
	signal MPMC_data_X : std_logic_vector(31 downto 0);
	signal MPMC_data_Y : std_logic_vector(31 downto 0);
	signal dpram_data_o : std_logic_vector(31 downto 0);
	signal dpram_addr_o : std_logic_vector(9 downto 0);
	signal dpram_wr_o : std_logic;
	signal pkt_head_info : std_logic_vector(9 downto 0);
	signal CWReStateDbg : std_logic_vector(5 downto 0);
	signal CCWReStateDbg : std_logic_vector(5 downto 0);
	signal CW_RxUsrClock : std_logic;
	signal CCW_RxUsrClock : std_logic;
	signal CWRXDATAIn : std_logic_vector(31 downto 0);
	signal CCWRXDATAIn : std_logic_vector(31 downto 0);
	signal CWCharIsIn : std_logic;
	signal CCWCharIsIn : std_logic;
	signal TRACK_DATA_OUT0 : std_logic;
	signal TRACK_DATA_OUT1 : std_logic;
	signal q0_clk1_refclk_o : std_logic;
   

	signal    trig_WD_timeout_cnt  : std_logic_vector(31 downto 0);
	signal    CW_CRCErrorCount  : std_logic_vector(31 downto 0);
	signal    CCW_CRCErrorCount  : std_logic_vector(31 downto 0);     
	signal    CW_LocalLoopbackDataErrorCnt  : std_logic_vector(31 downto 0);
	signal    CCW_LocalLoopbackDataErrorCnt  : std_logic_vector(31 downto 0);    
	signal    CW_LocalTxRxFrameErrorVal  : std_logic_vector(31 downto 0);
	signal    CCW_LocalTxRxFrameErrorVal  : std_logic_vector(31 downto 0);      
	signal    CwRemoteTimeoutCnt  : std_logic_vector(31 downto 0);
	signal    CcwRemoteTimeoutCnt  : std_logic_vector(31 downto 0);                  
	signal    CW_ReLocalHead_WD_timeout_cnt  : std_logic_vector(31 downto 0);     
	signal    CCW_ReLocalHead_WD_timeout_cnt  : std_logic_vector(31 downto 0);     
	signal    CW_wd_LocalTxPacketTimeout_cnt  : std_logic_vector(31 downto 0);     
	signal    CCW_wd_LocalTxPacketTimeout_cnt  : std_logic_vector(31 downto 0);         
	signal    EvrTrigCnt  : std_logic_vector(31 downto 0);
	signal    CW_LinkOfLockErrorCnt  : std_logic_vector(31 downto 0);
	signal    CCW_LinkOfLockErrorCnt  : std_logic_vector(31 downto 0);
	signal    CW_RcvPktMask_WD_timeout_cnt  : std_logic_vector(31 downto 0);
	signal    CCW_RcvPktMask_WD_timeout_cnt  : std_logic_vector(31 downto 0);
	signal    DRAM_ReadStart_WD_cnt  : std_logic_vector(31 downto 0);
   
   
   
   
-------------------------------------------------------
--   --debug signals (connect to ila)
   attribute mark_debug     : string;
   attribute mark_debug of iobus_addr: signal is "true";
   attribute mark_debug of iobus_cs: signal is "true";  
   attribute mark_debug of iobus_rnw: signal is "true";
   attribute mark_debug of iobus_wrdata: signal is "true";
   attribute mark_debug of iobus_rddata: signal is "true";
   attribute mark_debug of sys_rst: signal is "true";
   
   --attribute mark_debug of adc_clk: signal is "true";
   
   attribute mark_debug of inttrig_enb : signal is "true";   
   attribute mark_debug of tbt_gate : signal is "true";
   attribute mark_debug of tbt_trig  :signal is "true";
   --attribute mark_debug of pt_trig : signal is "true";
   attribute mark_debug of fa_trig  :signal is "true";   
   attribute mark_debug of sa_trig  :signal is "true";  
   
   attribute mark_debug of evr_tbt_trig : signal is "true";  
   attribute mark_debug of evr_fa_trig : signal is "true"; 
   attribute mark_debug of evr_sa_trig : signal is "true"; 
   attribute mark_debug of evr_gps_trig : signal is "true"; 
   attribute mark_debug of evr_dma_trig : signal is "true"; 
   
   attribute mark_debug of soft_trig: signal is "true";
   attribute mark_debug of dma_trig: signal is "true";

   
   
   attribute mark_debug of evrTrigOut : signal is "true";
   
   
   
--   attribute mark_debug of ad9510_we : signal is "true";
--   attribute mark_debug of ad9510_data: signal is "true";
--   attribute mark_debug of ad9510_sclk_i : signal is "true";
--   attribute mark_debug of ad9510_sdata_i: signal is "true";
--   attribute mark_debug of ad9510_lat_i : signal is "true";

   

   attribute mark_debug of adca_raw: signal is "true";
   attribute mark_debug of adcb_raw: signal is "true";
   attribute mark_debug of adcc_raw: signal is "true";
   attribute mark_debug of adcd_raw: signal is "true"; 
   --attribute mark_debug of adca_filt: signal is "true";
   --attribute mark_debug of adcb_filt: signal is "true";
   --attribute mark_debug of adcc_filt: signal is "true";
   --attribute mark_debug of adcd_filt: signal is "true";    
   --attribute mark_debug of but_coslkup: signal is "true";
--   attribute mark_debug of but_sinlkup: signal is "true";     
--   attribute mark_debug of tbt_sum: signal is "true";
--   attribute mark_debug of tbt_cha: signal is "true";
--   attribute mark_debug of tbt_chb: signal is "true";   
--   attribute mark_debug of tbt_chc: signal is "true";  
--   attribute mark_debug of tbt_chd: signal is "true";   
   
--   attribute mark_debug of sa_cha: signal is "true";
--   attribute mark_debug of sa_chb: signal is "true";   
--   attribute mark_debug of sa_chc: signal is "true";  
--   attribute mark_debug of sa_chd: signal is "true";      
   
--   attribute mark_debug of dsp_filt_bypass: signal is "true";  
--   attribute mark_debug of cha_gain: signal is "true";     
--   attribute mark_debug of chb_gain: signal is "true";     
--   attribute mark_debug of chc_gain: signal is "true";     
--   attribute mark_debug of chd_gain: signal is "true";    
   
   attribute mark_debug of adc_trig_count: signal is "true";          
   attribute mark_debug of evr_ts: signal is "true";      
   
   attribute mark_debug of hs_axi_tx_active: signal is "true";
   attribute mark_debug of hs_tx_active: signal is "true";   
   attribute mark_debug of hs_ddrbaseaddr: signal is "true";
   attribute mark_debug of hs_ddrcuraddr: signal is "true"; 
   attribute mark_debug of hs_ddrbuflen: signal is "true";     
   attribute mark_debug of hs_fifo_rdcnt: signal is "true";
   attribute mark_debug of hs_fifo_rddata: signal is "true";
   attribute mark_debug of hs_fifo_rden: signal is "true";
   

   
begin

ad9510_sclk <= ad9510_sclk_i;
ad9510_sdata <= ad9510_sdata_i;
ad9510_lat <= ad9510_lat_i;

-- Simulate ADC clock
--adc_clk <= sys_clk;


--external tbt clk
tbt_clk_inst  : IBUFDS  port map (O => tbt_extclk, I => tbt_clk_p, IB => tbt_clk_n); 
--tbt_clk_inst  : IBUFDS  port map (O => open, I => tbt_clk_p, IB => tbt_clk_n); 


 -- output evr recovered clock 
evr_clk_inst  : OBUFDS  port map (O => evr_rcvd_clk_p, OB => evr_rcvd_clk_n, I => evr_rcvd_clk);
 
-- tri-state buffers for i2c interface from PS for si5338  i2c
i2c1_scl_buf : IOBUF port map (O=>i2c1_scl_i, IO=>si5338_i2c_scl, I=>i2c1_scl_o, T=>i2c1_scl_t);
i2c1_sda_buf : IOBUF port map (O=>i2c1_sda_i, IO=>si5338_i2c_sda, I=>i2c1_sda_o, T=>i2c1_sda_t);

-- SFP 0
i2c_evr_sfp0_scl_buf : IOBUF port map (O=>iic_rtl_scl_i, IO=>open, I=>iic_rtl_scl_o, T=>iic_rtl_scl_t);
i2c_evr_sfp0_sda_buf : IOBUF port map (O=>iic_rtl_sda_i, IO=>open, I=>iic_rtl_sda_o, T=>iic_rtl_sda_t);


		
		
		
--	signal iic_sel : STD_LOGIC_VECTOR ( 0 to 0 );

	
sfp_leds(0) <= evr_gps_trig;
sfp_leds(11 downto 1) <= iobus_leds(23 downto 13);
dbg_leds <= iobus_leds(11 downto 8);


fp_leds(7) <= dma_trig_stretch;	--sa_trig_stretch;
fp_leds(6) <= hs_tx_active;	--tbt_hs_tx_enb;
fp_leds(5) <= tbt_hs_tx_enb;	--hs_tx_active;
fp_leds(4) <= fa_hs_tx_enb;	--dma_trig_stretch;
fp_leds(3) <= sa_trig_stretch;		-- 10 Hz
fp_leds(2) <= hs_status(0);
fp_leds(1) <= '0';
fp_leds(0) <= ad9510_status;

--fp_leds(3 downto 0) <= adc_sat;


fp_out(0) <= tbt_gate1(0);		--tbt_extclk;
fp_out(1) <= pt_lo_gate_en(0);	--adc_clk;
	

	  
dbg(0) <= tbt_gate1(0);	--TxClock125MHz;	--sys_clk; 
dbg(1) <= tbt_trig(0);	--q0_clk1_refclk_o; 
dbg(2) <= tbt_gate2(0);	--fa_hs_fifo_rden;	--CW_RxUsrClock;	--adc_clk;
dbg(3) <= tbt_extclk; 
dbg(4) <= tbt_fa_ddr_data_valid;	--evr_tbt_trig;
dbg(5) <= tbt_ddr_data_valid;		--evr_fa_trig;
dbg(6) <= fa_ddr_data_valid;		--evr_sa_trig;
dbg(7) <= tbt_hs_fifo_rden;		--evr_gps_trig;
dbg(8) <= tbt_trig_src(0);	--CWCharIsIn;		--clk200mhz; --fp_in(0);
dbg(9) <= CCWCharIsIn;		--sa_trig; --fp_in(1);

dbg(10) <= evrTrigOut(0);	--CWRemoteModeDone_o;		--tbt_ddr_data_valid;	--evrTrigOut(0);
dbg(11) <= evrTrigOut(1);	--CCWRemoteModeDone_o;		--tbt_hs_fifo_rden; --evrTrigOut(1);
dbg(12) <= evrTrigOut(2); 	--CCWRemoteModeState;		--tbt_hs_tx_enb;	--evrTrigOut(2);
dbg(13) <= evrTrigOut(3);	--CWRemoteModeState;		--tbt_hs_tx_active;	--evrTrigOut(3);
dbg(14) <= CWLocalModeState;		--tbt_hs_axi_tx_active;	--evrTrigOut(4);
dbg(15) <= CCWLocalModeState;		--tbt_hs_fifo_empty;	--evrTrigOut(5);
dbg(16) <= CCW_LinkDataValid;				--evr_dma_trig;
dbg(17) <= sa_trig_stretch;	--LinkDataValid;					--dma_trig;
dbg(18) <= fa_trig(0);	--sdi_evr_fa_trig;					--hs_fifo_empty;
dbg(19) <= sa_trig(0);	--CWLocalDataValid;				--hs_fifo_rden;
--
dbg(20) <= '0'; 	-- downto 16) <= (others => '0');



lmx2541_spice       <= '1';    
lmx2541_rfout_en    <= '1';  
S00_AXIS_tvalid     <= '0';


-- system status
sys_status(3 downto 0) <= adc_sat(3 downto 0);
sys_status(4) <= ad9510_status;


sys_rst <= sys_rstb(0);

		
-- reads in ADC data
adc_inst :  readadcs
    port map(
        sys_rst => sys_rst, 	  
		adc_clk_p => adc_clk_p, 
		adc_clk_n => adc_clk_n, 	
		adc_clk_stopped => adc_clk_stopped,
		adc_clk_locked => adc_clk_locked, 		
		adc0_clk_p => adc0_clk_p, 
		adc0_clk_n => adc0_clk_n, 	 
        adc0_data_p => adc0_data_p, 
        adc0_data_n => adc0_data_n, 
		adc0_ovr_p => adc0_ovr_p, 
		adc0_ovr_n => adc0_ovr_n, 	 			  
		adc1_clk_p => adc1_clk_p, 
		adc1_clk_n => adc1_clk_n, 	 
        adc1_data_p => adc1_data_p, 
        adc1_data_n => adc1_data_n, 
		adc1_ovr_p => adc1_ovr_p, 
		adc1_ovr_n => adc1_ovr_n, 	
		adc2_clk_p => adc2_clk_p, 
		adc2_clk_n => adc2_clk_n, 	 
        adc2_data_p => adc2_data_p, 
        adc2_data_n => adc2_data_n, 
		adc2_ovr_p => adc2_ovr_p, 
		adc2_ovr_n => adc2_ovr_n, 	 			  
		adc3_clk_p => adc3_clk_p, 
		adc3_clk_n => adc3_clk_n, 	 
        adc3_data_p => adc3_data_p, 
        adc3_data_n => adc3_data_n, 
		adc3_ovr_p => adc3_ovr_p, 
		adc3_ovr_n => adc3_ovr_n, 
		
		adc_fifo_wren => ('1'), --adc_fifo_wren, 
		adc_dbg_sel => adc_dbg_sel(1 downto 0), 	
		dds_din => dds_adcsim_din,
		dds_we => dds_adcsim_we,           
			  
	    adc_clk_o => adc_clk,	-- adc clock 117 MHz
	    adc_sat => adc_sat,
		adca_data => adcd_raw,	--adca_raw,  
		adcb_data => adcb_raw,  
		adcc_data => adcc_raw,  
		adcd_data => adca_raw, --adcd_raw, 
		adc_fifo_empty => adc_fifo_empty
		
		);


-- provides dsp trigger signals
u_dsp_cntrl : dsp_cntrl 
	port map(
		adc_clk     => adc_clk,               
		tbt_extclk  => tbt_extclk,
		reset       => sys_rst,
		machine_sel => ("101"), 
		--tbt_gate_dly => (x"00000010"),  --tbt_gate_dly, 
		--tbt_gate_width => (x"0080"), --tbt_gate_width,
		tbt_gate_dly   	=> tbt_gate_dly, 
		tbt_gate_width 	=> tbt_gate_width,		
        inttrig_enb => inttrig_enb, 
        evrsync_cnt => '0', --evr_trig2, 
        evr_fa_trig => evr_fa_trig,
        evr_sa_trig => evr_sa_trig,       			
		tbt_gate    => tbt_gate,
        tbt_trig    => tbt_trig(0),
        pt_trig     => pt_trig,
		fa_trig     => fa_trig(0), 
        sa_trig     => sa_trig(0),
        sa_count    => sa_count
		);  

u_dsp_cntrl_gate2 : dsp_cntrl 
	port map(
		adc_clk        	=> adc_clk,               
		tbt_extclk     	=> tbt_extclk,
		reset		    => sys_rst,
		machine_sel     => ("101"), 
		tbt_gate_dly   	=> x"0000" & tbt_gate2_ctrl(15 DOWNTO 0), 
		tbt_gate_width 	=> tbt_gate2_ctrl(31 DOWNTO 16),
        inttrig_enb     => inttrig_enb, 
        evrsync_cnt     => '0',
        evr_fa_trig     => evr_fa_trig,
        evr_sa_trig     => evr_sa_trig,                    			
		tbt_gate        => tbt_gate2(0),
        tbt_trig		=> tbt_trig2,
        pt_trig         => open,
		fa_trig         => open, 
        sa_trig         => open,
		sa_count        => open
		); 
		
		tbt_gate1(0)    <= tbt_gate;
		tbt_trig_src(0) <= tbt_trig2  when ( adc_dbg_sel(5) = '1' ) else tbt_trig(0);
		
		
-- PT gate control		
u_dsp_cntrl_pt_gate : dsp_cntrl 
	port map(
		adc_clk        	=> adc_clk,               
		tbt_extclk     	=> tbt_extclk,
		reset		    => sys_rst,
		machine_sel     => ("101"), 
		tbt_gate_dly   	=> x"0000" & pt_tbt_gate_ctrl(15 DOWNTO 0), 
		tbt_gate_width 	=> pt_tbt_gate_ctrl(31 DOWNTO 16),
        inttrig_enb     => inttrig_enb, 
        evrsync_cnt     => '0',
        evr_fa_trig     => evr_fa_trig,
        evr_sa_trig     => evr_sa_trig,                    			
		tbt_gate        => pt_lo_gate_en(0),
        tbt_trig		=> open,
        pt_trig         => open,
		fa_trig         => open, 
        sa_trig         => open,
		sa_count        => open
		); 

		
--AD9510 PLL: generates adc_clk
pll: spi_ad9510 
    port map(
        clk => sys_clk,
        reset => sys_rst, 
	    we => ad9510_we, 
	    data => ad9510_data, 
        sclk => ad9510_sclk_i, 
	    sdo	=> ad9510_sdo, 		--input from ad9510
        sdi => ad9510_sdata_i,
        csb => ad9510_lat_i, 
        debug => ad9510_debug
  );  


--pilot tone synthesizer spi interface
pt: spi_lmx2541
    port map(
        clk => sys_clk,                       
        reset => sys_rst,                      
	    we => lmx2541_we,
	    data => lmx2541_data,
        sclk => lmx2541_spiclk,                        
        sdata => lmx2541_spidata, 
        sle => lmx2541_spistrobe,
        ce => open);   --lmx2541_spice                       


--programmable attenuator for rf chain  
atten0: spi_pe43701 
    port map(
        clk => sys_clk,                  
        reset => sys_rst,                         
        strobe => dsa0_we,
	    wrdata => dsa_data,
        sclk => dsa0_clk,                         
        sdi => dsa0_sdata, 
        csb => dsa0_latch,                         
        debug => dsa0_debug);    


--programmable attenuator for pilot tone synthesizer
atten2: spi_pe43701 
    port map(
        clk => sys_clk,                  
        reset => sys_rst,                         
	    strobe => dsa2_we, 
	    wrdata => dsa_data,
        sclk => dsa2_clk,                         
        sdi => dsa2_sdata, 
        csb => dsa2_latch,                         
        debug => dsa0_debug);    


-- i/o registers to zynq processor
iobus_io : iobus_interface 
    generic map (
        FPGA_VERSION => FPGA_VERSION,
        VIVADO_VERSION => VIVADO_VERSION
        )
    port map ( 
		clk => sys_clk,
        rst => sys_rst, 	 
        addr => iobus_addr,
        cs => iobus_cs, 
        rnw => iobus_rnw, 
        wrdata => iobus_wrdata, 
        rddata => iobus_rddata, 		
        
        sysgen_version => sysgen_version,	  
		tbt_gate2_ctrl => tbt_gate2_ctrl,	
        adc_fifo_wren => adc_fifo_wren,
        adc_dbg_sel => adc_dbg_sel,
        
        hs_burstlen => hs_burstlen, 
        hs_cntrl => hs_cntrl, 
        hs_status => '0' & fa_hs_tx_enb & tbt_hs_tx_enb & hs_status(0), 
        hs_ddrbaseaddr => hs_ddrbaseaddr, 
        hs_ddrcuraddr => hs_ddrcuraddr,
        hs_ddrbuflen => hs_ddrbuflen,
        hs_throttle => hs_throttle,       
               
        ad9510_data => ad9510_data,
        ad9510_we => ad9510_we, 
        
        lmx2541_we => lmx2541_we, 
        lmx2541_data => lmx2541_data,
        lmx2541_rfenb => lmx2541_rfenb, 
        
        dsa_data => dsa_data, 
        dsa0_we => dsa0_we, 
        dsa2_we => dsa2_we,         
        
        inttrig_enb => inttrig_enb,  
        dsp_filt_bypass => dsp_filt_bypass,  
        
        soft_trig => soft_trig,
        
        kx => kx, 
        ky => ky, 
        cha_gain => cha_gain, 
        chb_gain => chb_gain, 
        chc_gain => chc_gain, 
        chd_gain => chd_gain, 
        xpos_offset => xpos_offset, 
        ypos_offset => ypos_offset,    
        
        evr_mapping_ram => evr_mapping_ram,
        evr_mapping_ram_we => evr_mapping_ram_we,
        evr_ts => evr_ts, 
        evr_ts_trig => evr_ts_trig,
            
        trig_count => adc_trig_count, 
        
		sys_status  => sys_status,	-- system status

		-- sa
        sa_count      => sa_count, 
        sa_cha        => sa_cha,     
        sa_chb        => sa_chb,      
        sa_chc        => sa_chc,                     
        sa_chd        => sa_chd,            
        sa_xpos       => sa_xpos,                   
        sa_ypos       => sa_ypos,                    
        sa_sum        => sa_sum, 
        sa_q          => but_q_sa,        
		-- gate2 sa
 		but_va_sa2    => but_va_sa2,
		but_vb_sa2    => but_vb_sa2,
		but_vc_sa2    => but_vc_sa2,
		but_vd_sa2    => but_vd_sa2,
		but_sum_sa2   => but_sum_sa2,
		but_xpos_sa2  => but_xpos_sa2,
		but_ypos_sa2  => but_ypos_sa2,
		-- pt hi/lo sa		
		pt_va_sa_hi   => pt_va_sa_hi, 
		pt_vb_sa_hi   => pt_vb_sa_hi, 
		pt_vc_sa_hi   => pt_vc_sa_hi, 
		pt_vd_sa_hi   => pt_vd_sa_hi, 
		pt_va_sa_lo   => pt_va_sa_lo, 
		pt_vb_sa_lo   => pt_vb_sa_lo, 
		pt_vc_sa_lo   => pt_vc_sa_lo, 
		pt_vd_sa_lo   => pt_vd_sa_lo, 
		-- temperature senser
        dfe_temp0           => dfe_temp0,
        dfe_temp1           => dfe_temp1,
        dfe_temp2           => dfe_temp2,
        dfe_temp3           => dfe_temp3,
        afe_temp0           => afe_temp0,
        afe_temp1           => afe_temp1,
				
		-- TBT
		tbt_hs_ddr_baseaddr  => tbt_hs_ddr_baseaddr,
		tbt_hs_ddr_buflen   => tbt_hs_ddr_buflen,
		tbt_hs_burst_len	=> tbt_hs_burst_len,
		
		-- FA
		fa_hs_ddr_baseaddr  => fa_hs_ddr_baseaddr,
		fa_hs_ddr_buflen    => fa_hs_ddr_buflen,
		fa_hs_burst_len	    => fa_hs_burst_len,		
	
		npi_trig_dly        => npi_trig_dly, 
		npi_trig_sel        => npi_trig_sel, 
		sdi_ctrl            => sdi_ctrl,
		
		sdi_device_addr		=> sdi_device_addr, 
		sdi_packet_len		=> sdi_packet_len, 		
		
		Sdi_WfmFreq			=> Sdi_WfmFreq,  
		Sdi_Wfm_Kx			=> Sdi_Wfm_Kx,   
		Sdi_Wfm_Ky			=> Sdi_Wfm_Ky,   			

		tbt_gate_dly		=> tbt_gate_dly, 
		tbt_gate_width		=> tbt_gate_width, 

		evr_width_trig0     => evr_width_trig0,
		
		--
        sfp_i2c_trig        => sfp_i2c_trig, 
        sfp_i2c_addr        => sfp_i2c_addr, 
        sfp0_i2c_data       => sfp0_i2c_data, 
        sfp1_i2c_data       => sfp1_i2c_data,     
        sfp2_i2c_data       => sfp2_i2c_data,     
        sfp3_i2c_data       => sfp3_i2c_data, 
        sfp4_i2c_data       => sfp4_i2c_data,
        sfp5_i2c_data       => sfp5_i2c_data, 
		
		tbt_hs_ddr_curaddr  => tbt_hs_ddr_curaddr,
		fa_hs_ddr_curaddr   => fa_hs_ddr_curaddr,
		
		
		test_reg0	        => test_reg0,
        test_reg1           => test_reg1,
		
        pT_LoDpramData      => pT_LoDpramData,
        pT_LoDpramAddr      => pT_LoDpramAddr,
		-- PT low phase slow
		but_ptLoPhase_sA	=> but_ptLoPhase_sA,
		but_ptLoPhase_sB	=> but_ptLoPhase_sB,
		but_ptLoPhase_sC	=> but_ptLoPhase_sC,
		but_ptLoPhase_sD	=> but_ptLoPhase_sD,	

		--	button phase
		but_pha_sa	=> but_pha_sa,	
		but_pha_sb	=> but_pha_sb,	
		but_pha_sc	=> but_pha_sc,	
		but_pha_sd	=> but_pha_sd,
		but_phase_Q_sa	=> but_phase_Q_sa,
		but_phase_I_sa	=> but_phase_I_sa,
		
		
		pt_sa_gain_a => pt_sa_gain_a,
		pt_sa_gain_b => pt_sa_gain_b,
		pt_sa_gain_c => pt_sa_gain_c,
		pt_sa_gain_d => pt_sa_gain_d,
		pt_tbt_gate_ctrl => pt_tbt_gate_ctrl,

       		
		sdi_EvrTrigCnt                   => EvrTrigCnt,	
		sdi_trig_WD_timeout_cnt  		 => trig_WD_timeout_cnt,
		sdi_CW_CRCErrorCount  			 => CW_CRCErrorCount,
		sdi_CCW_CRCErrorCount  			 =>  CCW_CRCErrorCount,
		sdi_CW_LocalLoopbackDataErrorCnt  => CW_LocalLoopbackDataErrorCnt,
		sdi_CCW_LocalLoopbackDataErrorCnt => CCW_LocalLoopbackDataErrorCnt,
		sdi_CW_LocalTxRxFrameErrorVal  	=> CW_LocalTxRxFrameErrorVal,
		sdi_CCW_LocalTxRxFrameErrorVal  => CCW_LocalTxRxFrameErrorVal,
		sdi_CwRemoteTimeoutCnt   => CwRemoteTimeoutCnt,
		sdi_CcwRemoteTimeoutCnt  =>	CcwRemoteTimeoutCnt,	 	 	
		sdi_CW_ReLocalHead_WD_timeout_cnt  => CW_ReLocalHead_WD_timeout_cnt,
		sdi_CCW_ReLocalHead_WD_timeout_cnt  => 	CCW_ReLocalHead_WD_timeout_cnt,
		sdi_CW_wd_LocalTxPacketTimeout_cnt  => 	CW_wd_LocalTxPacketTimeout_cnt,
		sdi_CCW_wd_LocalTxPacketTimeout_cnt  => 	CCW_wd_LocalTxPacketTimeout_cnt,			
		sdi_CW_LinkOfLockErrorCnt  => CW_LinkOfLockErrorCnt,
		sdi_CCW_LinkOfLockErrorCnt  => CCW_LinkOfLockErrorCnt,
		sdi_CW_RcvPktMask_WD_timeout_cnt  => CW_RcvPktMask_WD_timeout_cnt,
		sdi_CCW_RcvPktMask_WD_timeout_cnt  => CCW_RcvPktMask_WD_timeout_cnt,
		sdi_DRAM_ReadStart_WD_cnt  => DRAM_ReadStart_WD_cnt,		
		

		leds => iobus_leds
		);


	-- PT LO DPRAM lookup table controle	
    pt_ram_data   <= pT_LoDpramData;
    pt_ram_addr   <= pT_LoDpramAddr( 9 downto 0 );
    pt_ram_sin_we(0) <= pT_LoDpramAddr(10);
    pt_ram_cos_we(0) <= pT_LoDpramAddr(11);
	
	
-- DMA data port
--dma_trig <= soft_trig or evr_dma_trig;
dma_trig <= ddr_trig_start;
pt_lo_nco_count16 <= "000000" & pt_lo_nco_count;	

	
adca_data   <= adca_raw         when (adc_dbg_sel(4 downto 2) = "000") else	
               adca_filt		when (adc_dbg_sel(4 downto 2) = "001") else	
               adc_cha_but2     when (adc_dbg_sel(4 downto 2) = "010") else  			   
			   pt_va_hi         when (adc_dbg_sel(4 downto 2) = "011") else
			   pt_va_lo_filt    when (adc_dbg_sel(4 downto 2) = "100") else
			   pt_lo_imag_cha   when (adc_dbg_sel(4 downto 2) = "101") else
			   pt_lo_nco_count16  when (adc_dbg_sel(4 downto 2) = "110") else			   			   
               pt_coslkup_lo;
			   
adcb_data   <= adcb_raw         when (adc_dbg_sel(4 downto 2) = "000") else	
               adcb_filt		when (adc_dbg_sel(4 downto 2) = "001") else	
               adc_chb_but2     when (adc_dbg_sel(4 downto 2) = "010") else  
			   pt_vb_hi         when (adc_dbg_sel(4 downto 2) = "011") else
			   pt_vb_lo_filt    when (adc_dbg_sel(4 downto 2) = "100") else
			   pt_lo_qmag_cha   when (adc_dbg_sel(4 downto 2) = "101") else
			   pt_lo_nco_count16  when (adc_dbg_sel(4 downto 2) = "110") else					   
			   pt_sinlkup_lo;
			   
adcc_data   <= adcc_raw         when (adc_dbg_sel(4 downto 2) = "000") else	
               adcc_filt		when (adc_dbg_sel(4 downto 2) = "001") else	
               adc_chc_but2     when (adc_dbg_sel(4 downto 2) = "010") else  
			   pt_vc_hi         when (adc_dbg_sel(4 downto 2) = "011") else
			   pt_vc_lo_filt    when (adc_dbg_sel(4 downto 2) = "100") else
			   pt_lo_imag_chb   when (adc_dbg_sel(4 downto 2) = "101") else
			   pt_lo_nco_count16  when (adc_dbg_sel(4 downto 2) = "110") else				   
               pt_coslkup;
			   
adcd_data   <= adcd_raw         when (adc_dbg_sel(4 downto 2) = "000") else	
               adcd_filt		when (adc_dbg_sel(4 downto 2) = "001") else	
               adc_chd_but2     when (adc_dbg_sel(4 downto 2) = "010") else  
			   pt_vd_hi         when (adc_dbg_sel(4 downto 2) = "011") else
			   pt_vd_lo_filt    when (adc_dbg_sel(4 downto 2) = "100") else
			   pt_lo_qmag_chb   when (adc_dbg_sel(4 downto 2) = "101") else
			   pt_lo_nco_count16  when (adc_dbg_sel(4 downto 2) = "110") else	
               pt_sinlkup;
			     
		
			   
adchsdata: adcdata2ddr 
    port map (
        sys_clk  => clk200mhz,	--sys_clk,
        adc_clk  => adc_clk,
        reset    => sys_rst,      
        
        trig     => dma_trig,
        fifo_rst => ddr_trig_start,		-- fifo reset
				
        adc_cha => adca_data,
        adc_chb => adcb_data,
        adc_chc => adcc_data,
        adc_chd => adcd_data,               

        hs_cntrl => hs_cntrl,
        hs_status => hs_status,
	    hs_burstlen => hs_burstlen,	 
	    hs_throttle => hs_throttle,
	    
	    trig_count     => adc_trig_count,
	 	  
        hs_fifo_rdcnt  => hs_fifo_rdcnt,
        hs_fifo_rddata => hs_fifo_rddata, 
        hs_fifo_empty  => hs_fifo_empty,
        hs_fifo_rden   => hs_fifo_rden, 
        hs_tx_enb      => hs_tx_enb,   
        hs_tx_active   => hs_tx_active     
        );    




-- Main DSP processing block, from System Generator
  
	---------------------------------------
	dsp_but_pt : dsp_but_phase_gate_pt_zynq2_0
    PORT MAP (
			
			clk => adc_clk,	-- adc clock 8.5 ns
			--tbt_trig => tbt_trig,
			tbt_trig => tbt_trig_src,
			fa_trig  => fa_trig,
			sa_trig  => sa_trig,
			machine_sel => ("101"),	 --machine_sel,			
			tbt_gate1 => tbt_gate1,	
			tbt_gate2 => tbt_gate2,	
			
			-- adc raw
			adc_cha =>	adca_raw,  
			adc_chb	=>	adcb_raw, 
			adc_chc	=>	adcc_raw, 
			adc_chd	=>	adcd_raw, 
			-- adc filter output
			adc_but_sum  => open, --adc_but_sum,
			adc_cha_but  => adca_filt,		
			adc_chb_but  => adcb_filt,		
			adc_chc_but  => adcc_filt,		
			adc_chd_but  => adcd_filt,
			-- gate2 ADC
			adc_cha_but2 => adc_cha_but2,
			adc_chb_but2 => adc_chb_but2,
			adc_chc_but2 => adc_chc_but2,			
			adc_chd_but2 => adc_chd_but2,
			
			--
			filt_bypass       => dsp_filt_bypass(0 downto 0),
			pt_lo_filt_bypass => dsp_filt_bypass(1 downto 1),
			-- gain
			gain_cha => cha_gain,
			gain_chb => chb_gain,
			gain_chc => chc_gain,
			gain_chd => chd_gain,
			beam => "1",
			kx   => kx,
			ky   => ky,
					
			tbt_sum_ref => (x"00000000"), --tbt_sum_ref,
					
			xoffset  => xpos_offset,
			xoffset2 => (x"00000000"), --xoffset2,
			yoffset  => ypos_offset,
			yoffset2 => (x"00000000"), --yoffset2,
						

			-- fa	
			but_q_fa    => but_q_fa,
			but_sum_fa  => but_sum_fa,
			but_va_fa   => but_va_fa,
			but_vb_fa   => but_vb_fa,
			but_vc_fa   => but_vc_fa,
			but_vd_fa   => but_vd_fa,
			but_xpos_fa => but_xpos_fa,
			but_ypos_fa => but_ypos_fa,
		
			--sa outputs      
			but_sum_sa  => sa_sum,
			but_va_sa   => sa_cha,
			but_vb_sa   => sa_chb,
			but_vc_sa   => sa_chc,
			but_vd_sa   => sa_chd,
			but_xpos_sa => sa_xpos,
			but_ypos_sa => sa_ypos,
			but_q_sa	=> but_q_sa,
			
			-- gate 2 fa	
			but_sum_fa2   => open, --but_sum_fa2,
			but_va_fa2    => but_va_fa2,
			but_vb_fa2    => but_vb_fa2,
			but_vc_fa2    => but_vc_fa2,
			but_vd_fa2    => but_vd_fa2,
			but_xpos_fa2  => but_xpos_fa2,
			
				
			-- gate 2 sa		
			but_sum_sa2 => but_sum_sa2,
			but_va_sa2 => but_va_sa2,
			but_vb_sa2 => but_vb_sa2,
			but_vc_sa2 => but_vc_sa2,
			but_vd_sa2 => but_vd_sa2,
			but_xpos_sa2 => but_xpos_sa2,
			but_ypos_sa2 => but_ypos_sa2,
		
				
			-- phase
			but_pha_sa => but_pha_sa,
			but_pha_sb => but_pha_sb,
			but_pha_sc => but_pha_sc,
			but_pha_sd => but_pha_sd,
			but_q_phase_sa =>  but_phase_Q_sa,	-- CH-A Q
			but_i_phase_sa =>  but_phase_I_sa,	-- CH-A I
			
			
			-- PT adc after filter
			pt_va => pt_va_hi,
			pt_vb => pt_vb_hi,
			pt_vc => pt_vc_hi,
			pt_vd => pt_vd_hi,
			
			-- pt tbt	

			pt_va_tbt => open, --pt_va_tbt,
			pt_vb_tbt => open, --pt_vb_tbt,
			pt_vc_tbt => open, --pt_vc_tbt,
			pt_vd_tbt => open, --pt_vd_tbt,
			-- pt sa
			pt_va_sa => pt_va_sa_hi, 
			pt_vb_sa => pt_vb_sa_hi, 
			pt_vc_sa => pt_vc_sa_hi, 
			pt_vd_sa => pt_vd_sa_hi, 
		
	
	
			but_coslkup => but_coslkup,
			but_sinlkup => but_sinlkup,
			-- tbp phase
			but_pha_tbt => open, --but_pha_tbt,
			but_phb_tbt => open, --but_phb_tbt,
			but_phc_tbt => open, --but_phc_tbt,
			but_phd_tbt => open, --but_phd_tbt,
			
			-- I&Q
			tbt_va_i1 => open, --tbt_va_i1,
			tbt_va_q1 => open, --tbt_va_q1,
			tbt_vc_i  => open, --tbt_vc_i,
			tbt_vc_q  => open, --tbt_vc_q,
			
			beam_dump => open, --beam_dump,
			beam_dump1 => open, --beam_dump1,
			but_q_tbt => open,		
		
			but_sum_tbt => tbt_sum,
			but_va_tbt => tbt_cha,
			but_vb_tbt => tbt_chb,
			but_vc_tbt => tbt_chc,
			but_vd_tbt => tbt_chd,
			
			but_xpos_nm_tbt => but_xpos_nm_tbt,
			but_xpos_tbt => open,	--but_xpos_tbt,
			but_ypos_nm_tbt => but_ypos_nm_tbt,
			but_ypos_tbt => open,	--but_ypos_tbt,
		
			--post mortem
			pm_tbt_sum   => open, --pm_tbt_sum,
			pm_tbt_x_raw => open, --pm_tbt_x_raw,
			pm_tbt_y_raw => open, --pm_tbt_y_raw,
			tbt_i16_a    => open, --tbt_i16_a,
			tbt_i16_b    => open, --tbt_i16_b,
			tbt_i16_c    => open, --tbt_i16_c,
			tbt_i16_d    => open, --tbt_i16_d,
		
			-- gate 2 tbt
			but_sum_tbt2     => but_sum_tbt2,
			but_va_tbt2      => but_va_tbt2,
			but_vb_tbt2      => but_vb_tbt2,
			but_vc_tbt2      => but_vc_tbt2,
			but_vd_tbt2      => but_vd_tbt2,
			but_xpos_nm_tbt2 => but_xpos_nm_tbt2,
			but_ypos_nm_tbt2 => but_ypos_nm_tbt2,
			
			-- tbt 16 bit
			tbt_i16_a2 => open, --tbt_i16_a2,
			tbt_i16_b2 => open, --tbt_i16_b2,
			tbt_i16_c2 => open, --tbt_i16_c2,
			tbt_i16_d2 => open, --tbt_i16_d2,
			
			
			
			pt_va_lo => pt_va_lo_filt, 
			pt_vb_lo => pt_vb_lo_filt, 
			pt_vc_lo => pt_vc_lo_filt, 
			pt_vd_lo => pt_vd_lo_filt, 

	
			pt_coslkup_lo => pt_coslkup_lo,
			pt_sinlkup_lo => pt_sinlkup_lo,
			
			pt_sinlkup	=> pt_sinlkup,
			pt_coslkup	=> pt_coslkup,
			
			-- PT LO TBT signal A, B, C, D
			pt_va_tbt_lo  => pt_va_tbt_lo,
			pt_vb_tbt_lo  => pt_vb_tbt_lo,
			pt_vc_tbt_lo  => pt_vc_tbt_lo,
			pt_vd_tbt_lo  => pt_vd_tbt_lo,
			
			-- PT LO slow 10 Hz
			pt_va_sa_lo   => pt_va_sa_lo,
			pt_vb_sa_lo   => pt_vb_sa_lo,
			pt_vc_sa_lo   => pt_vc_sa_lo,
			pt_vd_sa_lo   => pt_vd_sa_lo,
			
			-- PT low phase slow			
			pt_lo_va_phase_sa => but_ptLoPhase_sA,
			pt_lo_vb_phase_sa => but_ptLoPhase_sB,
			pt_lo_vc_phase_sa => but_ptLoPhase_sC,
			pt_lo_vd_phase_sa => but_ptLoPhase_sD,	
			
			--
			pt_ram_data   => pt_ram_data,     --: in STD_LOGIC_VECTOR ( 31 downto 0 );
			pt_ram_addr   => pt_ram_addr,     --: in STD_LOGIC_VECTOR ( 9 downto 0 );
			pt_ram_sin_we => pt_ram_sin_we,   --: in STD_LOGIC_VECTOR ( 0 to 0 );
			pt_ram_cos_we => pt_ram_cos_we,   --: in STD_LOGIC_VECTOR ( 0 to 0 );
	
	
			-- pt hi/lo			
			pt_lo_gate_en     => pt_lo_gate_en(0 to 0),
			-- tbt gain
			pt_sa_gain_a => pt_sa_gain_a,
			pt_sa_gain_b => pt_sa_gain_b,
			pt_sa_gain_c => pt_sa_gain_c,
			pt_sa_gain_d => pt_sa_gain_d,
			
			pt_lo_nco_count => pt_lo_nco_count,	-- lookup table address count
			
			-- demodulator output
			pt_lo_imag_cha => pt_lo_imag_cha,
			pt_lo_qmag_cha => pt_lo_qmag_cha,
			pt_lo_imag_chb => pt_lo_imag_chb,
			pt_lo_qmag_chb => pt_lo_qmag_chb,
  
			dsp_version   => sysgen_version
	
    );

	

	


--stretch the sa_trig signal so can be seen on LED
sa_led : stretch
	port map (
	    clk => adc_clk,
	  	reset => sys_rst, 
	  	sig_in => sa_trig(0), 
	  	len => 3000000, -- ~25ms;
	  	sig_out => sa_trig_stretch
);	  	
	  	
	  	
--stretch the sa_trig signal so can be seen on LED
dma_led : stretch
    port map (
        clk => sys_clk,
        reset => sys_rst, 
        sig_in => dma_trig, 
        len => 4000000, -- ~25ms;
        sig_out => dma_trig_stretch
);          
	  	
	  	

--embedded event receiver decoding
evr_fa_trig  <= evrTrigOut(0);
evr_sa_trig  <= evrTrigOut(1);
evr_gps_trig <= evrTrigOut(2);
evr_dma_trig <= evrTrigOut(3);
-- TODO evr_tbt_trig <= tbt comes from evr_datastream;

	  	
--embedded event receiver
evr: evr_top_module 
    port map (
        Reset => sys_rst,
        clk_60Mhz => sys_clk, --sysClk50M,
        GTREFCLK_PAD_N => gtx_evr_refclk_n,
        GTREFCLK_PAD_P => gtx_evr_refclk_p,
        RXN_IN => gtx_evr_rx_n,
        RXP_IN => gtx_evr_rx_p,
        axi_sysClk0 => sys_clk,
        mappingRamWrA   => evr_mapping_ram_we, --evr_mapping_ram(31),
        mappingRAM_Addr => evr_mapping_ram(23 downto 16),
        mappingRAM_Data => evr_mapping_ram(15 downto 0),
        trig0_dly => (x"00000002"),
        trig1_dly => (x"00000002"),
        trig2_dly => (x"00000002"),
        trig3_dly => (x"00000002"),
        trig4_dly => (x"00000002"),
        trig5_dly => (x"00000002"),
        trig6_dly => (x"00000002"),
        trig7_dly => (x"00000002"),
		--
        trig0_width => (evr_width_trig0),
        trig1_width => (x"00000100"),
        trig2_width => (x"002FAF08"),   --25ms width
        trig3_width => (x"00000100"),  
        trig4_width => (x"00000100"),
        trig5_width => (x"00000100"),
        trig6_width => (x"00000100"),
        trig7_width => (x"00000100"),
        TimeStamp => evr_ts,
        trig_out => evrTrigOut,
        eventClock => open,
        RfLockedClk125 => evr_rcvd_clk
);

   
-- dfe temperature sensors
digbrd_adt7410: adt7410_16bit     
    port map(
		clk => sys_clk,  
		reset => sys_rst, 
		scl => dfetemp_scl, 
		sda => dfetemp_sda,  
		temp0 => dfe_temp0,
		temp1 => dfe_temp1,
		temp2 => dfe_temp2,
		temp3 => dfe_temp3,
		debug => dfe_temp_debug
);    

-- afe temperature sensors
anabrd_adt7410 : adt7410_16bit 
  port map
  (
		clk        	    => sys_clk,  
		reset     	    => sys_rst, 
		scl			    => afetemp_scl, 
		sda			    => afetemp_sda,  
		temp0		 	=> afe_temp0,
		temp1		 	=> open,
		temp2		 	=> afe_temp1, 
		temp3		 	=> open,

		debug			=> afe_temp_debug
  );
  

	
	sfp_i2c_trig_src <= sdi_ctrl(30)  when (sdi_ctrl(31) = '1') else sfp_i2c_trig;
	
	sfp_iic : sfp_i2c_readout 
		port map(
		 clk                    => sys_clk,                      
		 reset                  => sys_rst,   
		 strobe                 => sfp_i2c_trig_src,	--sfp_i2c_trig, 
		 addr                   => sfp_i2c_addr,                   
		 sfp_sclk	            => sfp_sclk, 
		 sfp_sdata	            => sfp_sdata, 
		 sfp0_i2c_data          => sfp0_i2c_data,	-- evr
		 sfp1_i2c_data          => sfp1_i2c_data,	-- sdi 0
		 sfp2_i2c_data          => sfp2_i2c_data,	-- sdi 1
		 sfp3_i2c_data          => sfp3_i2c_data,             
		 sfp4_i2c_data          => sfp4_i2c_data,
		 sfp5_i2c_data          => sfp5_i2c_data     
	   );
	
	
-- zynq processor 
real_system:  if (SIM_MODE = 0) generate real_system_i: system
    port map (
        ddr_addr(14 downto 0) => ddr_addr(14 downto 0),
        ddr_ba(2 downto 0) => ddr_ba(2 downto 0),
        ddr_cas_n => ddr_cas_n,
        ddr_ck_n => ddr_ck_n,
        ddr_ck_p => ddr_ck_p,
        ddr_cke => ddr_cke,
        ddr_cs_n => ddr_cs_n,
        ddr_dm(3 downto 0) => ddr_dm(3 downto 0),
        ddr_dq(31 downto 0) => ddr_dq(31 downto 0),
        ddr_dqs_n(3 downto 0) => ddr_dqs_n(3 downto 0),
        ddr_dqs_p(3 downto 0) => ddr_dqs_p(3 downto 0),
        ddr_odt => ddr_odt,
        ddr_ras_n => ddr_ras_n,
        ddr_reset_n => ddr_reset_n,
        ddr_we_n => ddr_we_n,
        fixed_io_ddr_vrn => fixed_io_ddr_vrn,
        fixed_io_ddr_vrp => fixed_io_ddr_vrp,
        fixed_io_mio(53 downto 0) => fixed_io_mio(53 downto 0),
        fixed_io_ps_clk => fixed_io_ps_clk,
        fixed_io_ps_porb => fixed_io_ps_porb,
        fixed_io_ps_srstb => fixed_io_ps_srstb,
        IIC_1_scl_i => i2c1_scl_i, 
        IIC_1_scl_o => i2c1_scl_o, 
        IIC_1_scl_t => i2c1_scl_t, 
        IIC_1_sda_i => i2c1_sda_i, 
        IIC_1_sda_o => i2c1_sda_o, 
        IIC_1_sda_t => i2c1_sda_t,
        -- sfp0 i2c
		iic_rtl_scl_i => iic_rtl_scl_i,
		iic_rtl_scl_o => iic_rtl_scl_o,
		iic_rtl_scl_t => iic_rtl_scl_t,
		iic_rtl_sda_i => iic_rtl_sda_i,
		iic_rtl_sda_o => iic_rtl_sda_o,
		iic_rtl_sda_t => iic_rtl_sda_t,
		iic_sel       => iic_sel,		--oout STD_LOGIC_VECTOR ( 0 to 0 );

	
	
        sys_clk => sys_clk,
        sys_rst => sys_rstb,
        iobus_addr(15 downto 0) => iobus_addr(15 downto 0),      
        iobus_cs => iobus_cs,
        iobus_data_pl2ps(31 downto 0) => iobus_rddata(31 downto 0),
        iobus_data_ps2pl(31 downto 0) => iobus_wrdata(31 downto 0),
        iobus_rnw => iobus_rnw,
		--
		clk200mhz           => clk200mhz,
		fclk_clk2_25MHz     => fclk_clk2_25MHz,
        adc_hs_ddr_baseaddr => hs_ddrbaseaddr, 			    -- in
        adc_hs_ddr_curaddr  => hs_ddrcuraddr,               -- out STD_LOGIC_VECTOR ( 31 downto 0 );
        adc_hs_ddr_buflen   => hs_ddrbuflen,                -- in STD_LOGIC_VECTOR ( 8 downto 0 );
        adc_hs_fifo_rdcnt   => hs_fifo_rdcnt(8 downto 0),   -- in STD_LOGIC_VECTOR ( 8 downto 0 );
        adc_hs_fifo_rddata  => hs_fifo_rddata,	            -- in STD_LOGIC_VECTOR ( 63 downto 0 );
        adc_hs_fifo_rden    => hs_fifo_rden, 				-- out STD_LOGIC;
        adc_hs_tx_active    => hs_axi_tx_active,            -- out STD_LOGIC;
        adc_hs_tx_enb       => hs_tx_enb,					-- in STD_LOGIC;		
		
		
		-- TBT waveform		
		tbt_hs_ddr_baseaddr =>	tbt_hs_ddr_baseaddr,	-- in STD_LOGIC_VECTOR ( 31 downto 0 );
		tbt_hs_tx_enb 	    =>	tbt_hs_tx_enb,	    -- in OK
		tbt_hs_ddr_buflen   =>	tbt_hs_ddr_buflen,	-- in STD_LOGIC_VECTOR ( 31 downto 0 );
		tbt_hs_fifo_rdcnt   =>	tbt_hs_fifo_rdcnt,	-- in STD_LOGIC_VECTOR ( 8 downto 0 );
		tbt_hs_fifo_rddata  =>	tbt_hs_fifo_rddata,	-- in STD_LOGIC_VECTOR ( 63 downto 0 );	
		--	
		tbt_hs_ddr_curaddr  =>	tbt_hs_ddr_curaddr,	    -- out DDR offset, OK		
		tbt_hs_fifo_rden    =>	tbt_hs_fifo_rden,	    -- out NO output
		tbt_hs_tx_active    =>	tbt_hs_axi_tx_active,	-- out monitoring NO output

		fa_hs_ddr_baseaddr =>	fa_hs_ddr_baseaddr, -- in STD_LOGIC_VECTOR ( 31 downto 0 );
		fa_hs_ddr_buflen   =>	fa_hs_ddr_buflen, --  in STD_LOGIC_VECTOR ( 31 downto 0 );
		fa_hs_ddr_curaddr  =>	fa_hs_ddr_curaddr, --   out STD_LOGIC_VECTOR ( 31 downto 0 );
		fa_hs_fifo_rdcnt   =>	fa_hs_fifo_rdcnt, --  in STD_LOGIC_VECTOR ( 8 downto 0 );
		fa_hs_fifo_rddata  =>   fa_hs_fifo_rddata, --	 in STD_LOGIC_VECTOR ( 63 downto 0 );
		fa_hs_fifo_rden    =>	fa_hs_fifo_rden, --   out STD_LOGIC;
		fa_hs_tx_active    =>	fa_hs_axi_tx_active, --   out STD_LOGIC;
		fa_hs_tx_enb       =>	fa_hs_tx_enb, --   in STD_LOGIC		
		--
        tenhz_irq => sa_trig(0), --tenhz_irq,
        other_irq => other_irq,
		fa_irq    => fa_irq,
		tbt_irq   => tbt_irq
    );  
end generate;    
        

	  
simu_system: if (SIM_MODE = 1) generate sim_system_i: sim_system
    port map (
       sys_clk => sys_clk,    
       sys_rstb  => sys_rstb,
       iobus_addr => iobus_addr,
       iobus_cs => iobus_cs,
       iobus_rnw => iobus_rnw,
       iobus_wrdata => iobus_wrdata,
       iobus_rddata => iobus_rddata
     );
end generate;    

 
	-- trig timestamp
	trig_timestamp: trig_times PORT MAP (
		reset   => sys_rst,
		evr_clk => evr_rcvd_clk,
		adc_clk => adc_clk,
		trig_in => dma_trig,
		evr_ts_in => evr_ts,
		trig_ts   => evr_ts_trig
	);

   trig_soft : trig_edge_gen PORT MAP (
          clk      => adc_clk,
          trig_in  => soft_trig,                --(soft_trig OR evr_dma_trig),	--dma_trig,
          trig_out => soft_trig_out
        );

   trig_evr : trig_edge_gen PORT MAP (
          clk      => adc_clk,
          trig_in  => evr_dma_trig,             --(soft_trig OR evr_dma_trig),	--dma_trig,
          trig_out => evr_single_trig_out       --ddr_trig_start
        );

	
   tbt_hs_tx_enb_done_trig : trig_edge_gen PORT MAP (
          clk      => sys_clk,
          trig_in  => NOT tbt_hs_tx_enb,            
          trig_out => tbt_irq     
        );

   fa_hs_tx_enb_done_trig : trig_edge_gen PORT MAP (
          clk      => sys_clk,
          trig_in  => NOT fa_hs_tx_enb,            
          trig_out => fa_irq     
        );
	
	other_irq <= fa_irq;
	

	trig_cntrl : trig_sel 
	port map(
       adc_clk             => adc_clk,  
       reset               => sys_rst, 
       soft_trig           => soft_trig_out,	--npi_softtrig OR evr_soft_event_out,     -- internal
       evr_trig            => evr_single_trig_out,		-- ext
	   evr_soft_trig       => '0',				--evr_soft_event_out,			    -- soft event 6/24/2013
       fp_trig             => '0',              --fp_in(0),  -- OR evr_trig0 OR evr_trig4 OR evr_trig5, 	-- event trig 4 | 5 | lemo  1/25/17 for multiple event
       trig_sel            => npi_trig_sel, 	--(1 downto 0);
       trig_dly_reg        => npi_trig_dly, 
       npi_trig            => ddr_trig_start 	--npi_trig 
    );  
	 

	--	tbt waveform select
	tbt_wfm_a   <= tbt_cha  when (adc_dbg_sel(6) = '0') else pt_va_tbt_lo;			   
	tbt_wfm_b   <= tbt_chb  when (adc_dbg_sel(6) = '0') else pt_vb_tbt_lo;				   
	tbt_wfm_c   <= tbt_chc  when (adc_dbg_sel(6) = '0') else pt_vc_tbt_lo;				   
	tbt_wfm_d   <= tbt_chd  when (adc_dbg_sel(6) = '0') else pt_vd_tbt_lo;	

			   
    TBT : tbt_hp_ddr PORT MAP (
		adc_clk => adc_clk,
		rst => sys_rst,
		--tbt_trig => tbt_trig(0),	-- 378 kHz tbt
		tbt_trig => tbt_trig_src(0),
		endian_en => '1',
		but_va   => tbt_wfm_a, 	--tbt_cha,
		but_vb   => tbt_wfm_b, 	--tbt_chb,
		but_vc   => tbt_wfm_c,	--tbt_chc,
		but_vd   => tbt_wfm_d,	--tbt_chd,
		but_xpos => but_xpos_nm_tbt, 
		but_ypos => but_ypos_nm_tbt, 
		but_sum  => tbt_sum,

		-- Gate 2 TBT signal
		but_va2	=> but_va_tbt2,
		but_vb2 => but_vb_tbt2,
		but_vc2 => but_vc_tbt2,
		but_vd2 => but_vd_tbt2,
		
		-- PT LO TBT signal for PT algorithem developement
		--but_va2	=> pt_va_tbt_lo,
		--but_vb2 => pt_vb_tbt_lo,
		--but_vc2 => pt_vc_tbt_lo,
		--but_vd2 => pt_vd_tbt_lo,
		--
		ddr_data => tbt_ddr_data_in,
		ddr_data_valid => tbt_ddr_data_valid
        );


    FA : tbt_hp_ddr PORT MAP (
		adc_clk => adc_clk,
		rst => sys_rst,
		tbt_trig => fa_trig(0),	-- 378 kHz tbt
		endian_en => '1',
		but_va   => but_va_fa,
		but_vb   => but_vb_fa,
		but_vc   => but_vc_fa,
		but_vd   => but_vd_fa,
		but_xpos => but_xpos_fa,
		but_ypos => but_ypos_fa,
		but_sum  => but_sum_fa,

		but_va2	=> but_va_fa2,
		but_vb2 => but_vb_fa2,
		but_vc2 => but_vc_fa2,
		but_vd2 => but_vd_fa2,

		ddr_data => fa_ddr_data_in,
		ddr_data_valid => fa_ddr_data_valid
        );
		
	
	-- select tbt/fa waveform
	tbt_fa_ddr_data_in    <= tbt_ddr_data_in     when (sdi_ctrl(28) = '0') else fa_ddr_data_in;
	tbt_fa_ddr_data_valid <= tbt_ddr_data_valid  when (sdi_ctrl(28) = '0') else fa_ddr_data_valid;
	tbt_fa_hs_burst_len   <= tbt_hs_burst_len    when (sdi_ctrl(28) = '0') else fa_hs_burst_len;
	
	
    tbt_ddr : data2ddr_tf PORT MAP (
          sys_clk        => clk200mhz,
          adc_clk        => adc_clk,
          reset          => sys_rst,
          --trig           => dma_trig, --DDR data 2 clcok before
		  trig           => ddr_trig_start,
          burst_enb      => '1',
          burst_len      => tbt_fa_hs_burst_len,	--tbt_hs_burst_len, 
          testdata_en    => '1', 
		  
          --fifo_din_32bit => tbt_ddr_data_in,	-- in data   OK
          --fifo_din_valid => tbt_ddr_data_valid,	-- in        OK
		  --
          fifo_din_32bit => tbt_fa_ddr_data_in,	
          fifo_din_valid => tbt_fa_ddr_data_valid,	
			--		  
          hs_fifo_rdcnt  => tbt_hs_fifo_rdcnt,	-- out ok
          hs_fifo_rddata => tbt_hs_fifo_rddata,	-- out ( 63 downto 0 )
          hs_fifo_empty  => tbt_hs_fifo_empty,
          hs_fifo_rden   => tbt_hs_fifo_rden,	-- in 
          hs_fifo_rst    => ddr_trig_start,		-- fifo reset
          hs_tx_enb      => tbt_hs_tx_enb,		-- out ok
          hs_tx_active   => tbt_hs_tx_active,	-- out ok
          dbg_strobe_lat => tbt_hs_dbg_strobe_lat
    );
	
	
    fa_ddr : data2ddr_tf PORT MAP (
          sys_clk        => clk200mhz,
          adc_clk        => adc_clk,
          reset          => sys_rst,
          --trig           => dma_trig, --DDR data 2 clcok before
		  trig           => ddr_trig_start,
          burst_enb      => '1',
          burst_len      => fa_hs_burst_len, 
          testdata_en    => '1', 
		  
          fifo_din_32bit => fa_ddr_data_in,		-- in data   OK
          fifo_din_valid => fa_ddr_data_valid,	-- in        OK
		  --
          hs_fifo_rdcnt  => fa_hs_fifo_rdcnt,	-- out ok
          hs_fifo_rddata => fa_hs_fifo_rddata,	-- out ( 63 downto 0 )
          hs_fifo_empty  => fa_hs_fifo_empty,
          hs_fifo_rden   => fa_hs_fifo_rden,	-- in Not received
          hs_fifo_rst    => ddr_trig_start,		-- fifo reset
          hs_tx_enb      => fa_hs_tx_enb,		-- out ok
          hs_tx_active   => fa_hs_tx_active,	-- out ok
          dbg_strobe_lat => open
    );
		
		
	
	dbg_tbt_hs_fifo_rden(0) <= tbt_hs_fifo_rden; 
	dbg_tbt_hs_axi_tx_active(0) <= tbt_hs_axi_tx_active; 
	dbg_tbt_hs_tx_enb(0) <= tbt_hs_tx_enb;
	dbg_tbt_hs_tx_active(0) <= tbt_hs_tx_active; 
	dbg_tbt_hs_fifo_empty(0) <= tbt_hs_fifo_empty;
	dbg_tbt_ddr_data_valid(0) <= tbt_ddr_data_valid;
	dbg_ddr_trig_start(0) <= ddr_trig_start;
	dbg_tbt_hs_dbg_strobe_lat(0) <= tbt_hs_dbg_strobe_lat;
		
		
		
--	ila_tbt : ila_tbt_ddr
--	PORT MAP (
--		clk    => clk200mhz,
--		probe0 => tbt_ddr_data_in, 
--		probe1 => tbt_hs_fifo_rdcnt, 
--		probe2 => tbt_hs_fifo_rddata, 
--		probe3 => tbt_hs_ddr_curaddr, 
--		probe4 => dbg_tbt_hs_fifo_rden, 
--		probe5 => dbg_tbt_hs_axi_tx_active, 
--		probe6 => dbg_tbt_hs_tx_enb, 
--		probe7 => dbg_tbt_hs_tx_active, 
--		probe8 => dbg_tbt_hs_fifo_empty, 
--		probe9 => dbg_tbt_hs_dbg_strobe_lat, 
--		probe10 => dbg_tbt_ddr_data_valid,
--		probe11 => dbg_ddr_trig_start
--	);

	
--	ila_adc_data : ila_adc_raw
--	PORT MAP (
--		clk => adc_clk,
--		probe0 => adca_raw, 
--		probe1 => adcb_raw, 
--		probe2 => adcc_raw,
--		probe3 => adcd_raw
--	);
	
	
	-- SDI trigger
    sdi_trig : trig_edge_gen PORT MAP (
          clk      => sys_clk,
          trig_in  => evr_fa_trig,  -- 10 kHz
          trig_out => SdiTrig_i
        );

		
	fa_x_data :	sdi_data_forward port map ( clk_in => adc_clk, timestamp_in => but_xpos_fa, clk_out => evr_rcvd_clk, timestamp_out => sdi_fa_xpos );
	fa_y_data :	sdi_data_forward port map ( clk_in => adc_clk, timestamp_in => but_ypos_fa, clk_out => evr_rcvd_clk, timestamp_out => sdi_fa_ypos );
		
	-- SDI packet generator	
	bpm_local_sdi_data_gen : bpm_countdatagen        
	  	port map (
		sysClk			  => sys_clk,
		LtableRamInData   => x"00000000", --LookupTableRamInData,
		phaseCnt		  => x"00" & "00", --LookupTablePhaseCnt(9 DOWNTO 0),
		--
	  	Reset             =>  sys_rst, 
		--NCO_reset		  =>  evr_nco_rst,		-- reset by EVR
		NCO_reset		  =>  '0',
	  	sdi_clk           =>  evr_rcvd_clk, 	-- 125 MHz  
	  	Trigger           =>  evr_fa_trig,

		testMode          =>  sdi_ctrl(10 DOWNTO 7),
	  	maxCount          =>  "00000010" & "000000000001000000000000", 	  --4096
	  	fa_pos_x          =>  sdi_fa_xpos,	-- bpm fa x position data
	  	fa_pos_y          =>  sdi_fa_ypos,	-- bpm fa y position data
        DataSel		  	  =>  sdi_ctrl(6), 	-- FA data or internal sin wave ??
        wfmKx			  =>  Sdi_Wfm_Kx,	
        wfmKy			  =>  Sdi_Wfm_Ky,	  
        wfmPhase_inc	  =>  Sdi_WfmFreq,
		-- OUTPUT
        LocalDataValid    =>  CWLocalDataValid, --bpm_sim_dataValid, 
	  	LocalCountData    =>  bpm_SimLocalData, --bpm_sim_Data, 
	  	LocalBpmPosData   =>  CWLocalData,	-- to SDI input
	  	RampRst           =>  open,
	  	fa_evr_trig       =>  sdi_evr_fa_trig,
		TrigOut			  =>  open,
		glitch_out        =>  sdi_glitch_out,
		test_bit          =>  sdi_test_bit,
		sdi_fa_pos_x	  =>  sdi_fa_pos_x,	-- for PM
		sdi_fa_pos_y	  =>  sdi_fa_pos_y
	);		
		

		
-----////////////////////////////////////////////////-----


	sdi_LocalData      <= CWLocalData       when (sdi_ctrl(29) = '1') else bpm_SimLocalData;
	sdi_LocalDataValid <= CWLocalDataValid  when (sdi_ctrl(29) = '1') else CWLocalDataValid;

--sdi_comm : if ( SDI_ENABLE = 1) generate sdi_i: user_logic		
	
--    --sdi: user_logic 
--    PORT MAP (
--		clk_27M                  => fclk_clk2_25MHz,
--		sysClk                   => sys_clk,
--		SdiReset                 => sys_rst,
--		--
--		UserSdiReset             => sdi_ctrl(0),  --UserSdiReset,
--		gtx_refclk_n             => gtx_refclk_n,
--		gtx_refclk_p             => gtx_refclk_p,
--		RXN_IN                   => RXN_IN,
--		RXP_IN                   => RXP_IN,
--		TXN_OUT                  => open,
--		TXP_OUT                  => open,

--		SdiTrig_i                => evr_fa_trig,	--SdiTrig_i,		-- 10 kHz trigger for SDI Data transmitte
--		SdiUsrNpiTrigger         => '0', --SdiUsrNpiTrigger,
--		SdiUsrRstTrig            => sdi_ctrl(1), --SdiUsrRstTrig,
--		DeviceAddress            => sdi_device_addr(15 downto 0), --DeviceAddress,
--		PacketLength             => sdi_packet_len,   --PacketLength,
--		--Local packet Data  
--		CWLocalDataClock         => evr_rcvd_clk,	
--		CWLocalData              => sdi_LocalData,		--CWLocalData,
--		CWLocalDataValid         => sdi_LocalDataValid, --CWLocalDataValid,
--		CCWLocalData             => sdi_LocalData,		--CWLocalData,
--		CCWLocalDataValid        => sdi_LocalDataValid,	--CWLocalDataValid,


--		DpramOutMode             => sdi_ctrl(4), --DpramOutMode,
--		LinkOutDir               => sdi_ctrl(3 downto 2), --LinkOutDir,
--		DpRamTrigSrc             => sdi_ctrl(5),  --DpRamTrigSrc,
--		MaskContrl               => sdi_ctrl(27 downto 16), --MaskContrl,

--		MasterPacketAddress      => sdi_device_addr(31 downto 16),
--		StartCalc                => StartCalc,
--		DPRAM_ReadStart          => DPRAM_ReadStart,
--		DPRAM_Valid              => DPRAM_Valid,
--		DPRAM_WR_OUT             => DPRAM_WR_OUT,
--		axiBusClk                => axiBusClk,
--		axi_bus_en               => axi_bus_en,
--		axi_bus_addr             => axi_bus_addr,
--		axi_bus_data             => axi_bus_data,
--		NipStrobe                => NipStrobe,
--		NpiRst_Out               => NpiRst_Out,
--		TxClock125MHz            => TxClock125MHz,
--		LinkHead                 => LinkHead,
--		LinkData                 => LinkData,
--		LinkDataValid            => LinkDataValid,
--		LinkStartOfPacket        => LinkStartOfPacket,
--		CWLocalModeState         => CWLocalModeState,
--		CCWLocalModeState        => CCWLocalModeState,
--		CWRemoteModeState        => CWRemoteModeState,
--		CCWRemoteModeState       => CCWRemoteModeState,
--		sfp_link                 => sfp_link,
--		CW_LedRxFifoValid        => CW_LedRxFifoValid,
--		AllLinkValidStatus       => AllLinkValidStatus,
--		DirLED                   => DirLED,
--		CCW_LinkDataValid        => CCW_LinkDataValid,
--		BiDirLinkDataValid       => BiDirLinkDataValid,
--		RemotePacketDroppedStatus => RemotePacketDroppedStatus,
--		CWRemoteModeDone_o       => CWRemoteModeDone_o,
--		CCWRemoteModeDone_o      => CCWRemoteModeDone_o,
--		MyLocalLoopbackDataValid_o => MyLocalLoopbackDataValid_o,
--		CW_RcvPacketMaskStatus   => CW_RcvPacketMaskStatus,
--		CCW_RcvPacketMaskStatus  => CCW_RcvPacketMaskStatus,
--		ReceivedRemotePacketHead_s => ReceivedRemotePacketHead_s,
--		-- debug
--		CWSingleSdiDebugOut      => CWSingleSdiDebugOut,
--		CCWSingleSdiDebugOut     => CCWSingleSdiDebugOut,
--		--
--		MPMC_data_X              => MPMC_data_X,
--		MPMC_data_Y              => MPMC_data_Y,
--		--
--		dpram_data_o             => dpram_data_o,
--		dpram_addr_o             => dpram_addr_o,
--		dpram_wr_o               => dpram_wr_o,
--		--
--		pkt_head_info            => pkt_head_info,
--		CWReStateDbg             => CWReStateDbg,
--		CCWReStateDbg            => CCWReStateDbg,
--		CW_RxUsrClock            => CW_RxUsrClock,
--		CCW_RxUsrClock           => CCW_RxUsrClock,
--		CWRXDATAIn               => CWRXDATAIn,
--		CCWRXDATAIn              => CCWRXDATAIn,
--		CWCharIsIn               => CWCharIsIn,
--		CCWCharIsIn              => CCWCharIsIn,
--		TRACK_DATA_OUT0          => TRACK_DATA_OUT0,
--		TRACK_DATA_OUT1          => TRACK_DATA_OUT1,

--		EvrTrigCnt                  	=> EvrTrigCnt,	
--		trig_WD_timeout_cnt  			=> trig_WD_timeout_cnt,
--		CW_CRCErrorCount  				=> CW_CRCErrorCount,
--		CCW_CRCErrorCount  				=>  CCW_CRCErrorCount,
--		CW_LocalLoopbackDataErrorCnt  	=> CW_LocalLoopbackDataErrorCnt,
--		CCW_LocalLoopbackDataErrorCnt  	=> CCW_LocalLoopbackDataErrorCnt,
--		CW_LocalTxRxFrameErrorVal  		=> CW_LocalTxRxFrameErrorVal,
--		CCW_LocalTxRxFrameErrorVal  	=> CCW_LocalTxRxFrameErrorVal,
--		CwRemoteTimeoutCnt  			=> CwRemoteTimeoutCnt,
--		CcwRemoteTimeoutCnt  			=>	CcwRemoteTimeoutCnt,	 	 	
--		CW_ReLocalHead_WD_timeout_cnt  	=> CW_ReLocalHead_WD_timeout_cnt,
--		CCW_ReLocalHead_WD_timeout_cnt  => 	CCW_ReLocalHead_WD_timeout_cnt,
--		CW_wd_LocalTxPacketTimeout_cnt  => 	CW_wd_LocalTxPacketTimeout_cnt,
--		CCW_wd_LocalTxPacketTimeout_cnt => 	CCW_wd_LocalTxPacketTimeout_cnt,			
--		CW_LinkOfLockErrorCnt  			=> CW_LinkOfLockErrorCnt,
--		CCW_LinkOfLockErrorCnt  		=> CCW_LinkOfLockErrorCnt,
--		CW_RcvPktMask_WD_timeout_cnt  	=> CW_RcvPktMask_WD_timeout_cnt,
--		CCW_RcvPktMask_WD_timeout_cnt   => CCW_RcvPktMask_WD_timeout_cnt,
--		DRAM_ReadStart_WD_cnt  			=> DRAM_ReadStart_WD_cnt,
--		q0_clk1_refclk_o         		=> q0_clk1_refclk_o
--        );
--end generate; 
		
		
 
end behv;
