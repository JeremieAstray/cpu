--IDEntity.vhd- 译码模块
----------------------------------------------------------------------
--读取寄存器值和指令译码。采取一次译码，逐级传递的方式，译出后几级流水
--所需的控制信号和数据（如立即数等），在每次时钟上升沿到来时送入下一级
----------------------------------------------------------------------
--2004-09-04
library IEEE;
use IEEE.std_logic_1164.all; 
use IEEE.std_logic_unsigned.all;
use work.unitpack.all;

entity IDEntity is
	port(	reset			: in std_logic;
			clk				: in std_logic;	 			
			d_IR			: in word;	 
			d_PCInc1 		: in word;	 
			--*** for Regs Bank	 
			w_WBData  		: in word;	 			
			w_destReg 		: in std_logic_vector(1 downto 0);  -- Destination Reg index
			w_wRegEn  		: in std_logic;  					--寄存器写使能   			
			e_SA,e_SB		: out std_logic_vector(1 downto 0);	
			i_PCPlusOffset	: out word;
			--*** for ALU	 
			e_RAOut,e_RBOut	: out word;	 						--寄存器A,B输出
			e_IMM			: out word;	 						--立即数输出
			e_ALUSrc		: out std_logic_vector(2 downto 0); --运算器选源
			e_ALUOpr		: out std_logic_vector(3 downto 0); --运算器操作
			e_SetFlag		: out std_logic_vector(2 downto 0); --状态寄存器写使能
			--*** for Memory 
			e_wrMem	: out std_logic_vector(1 downto 0); --"00" write, "01" read, "1-" do nothing
			--*** for Regs
			e_wRegEn  : out  std_logic;		--寄存器写使能 
			e_destReg : out  std_logic_vector(1 downto 0); 
			e_MemToReg : out std_logic;	    --内存写入寄存器使能 			
			--Cpu Interface ---------------------------------
			RegSel 	: in std_logic_vector(1 downto 0);
			RegOut	: out word
			);
end entity ;

architecture IDArch of IDEntity is 
	SIGNAL RegArray			: REGISTERARRAY; 	-- Regs Array
    signal wRegIndex,ia,ib	: INDEXTYPE;	
	--译码生成的控制信号
	signal SA,SB			: std_logic_vector(1 downto 0);
	signal wrMem			: std_logic_vector(1 downto 0);
	signal wRegEn			: std_logic;
	signal memToReg			: std_logic;   
	signal offset			: word;
	signal RA,RB			: word;
	signal ALUSrc			: std_logic_vector(2 downto 0);
	signal ALUOpr			: std_logic_vector(3 downto 0);
	signal SetFlag			: std_logic_vector(2 downto 0);	  
	signal imm				: word;
begin 
	ia <= conv_integer(SA);
	ib <= conv_integer(SB);
	RA <= RegArray(ia);     --寄存器A口数据
	RB <= RegArray(ib);     --寄存器B口数据	
	--*******************************************
	--第五级流水,回写寄存器	
	wRegIndex <= conv_integer(w_destReg);
	WriteBack:process(reset,clk)   
	begin	   	
	    if reset='0' then
		    RegArray(0) <= x"00";
		    RegArray(2) <= x"00";
		elsif FALLING_EDGE(clk) and w_wRegEn='1' then
			RegArray(wRegIndex) <= w_WBData;
		end if;	 
	end process; 
	--*******************************************	
	--译码模块		
	Decode_Pro:process(d_IR) 
	variable op  : std_logic_vector(3 downto 0);
	variable ctrl:std_logic_vector(17 downto 0);
	begin 
		op := d_IR(7 downto 4);  --操作码 
	    --译码产生的信号:SA,SB,Wrmem,wRegEn,MemToReg,ALUSrc,ALUOpr,SetFlag   
		case op is
			when ADD	=> ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"001"&"0000"&"001";
			when SUBB 	=> ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"001"&"0001"&"001";
			when DEC    => ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"100"&"0000"&"001";
			when INC    => ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"010"&"0000"&"001";
			when ANDins => ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"001"&"0010"&"001";
			when CMP 	=> ctrl:=d_IR(3 downto 0)&"10"&"0"&"1"&"001"&"0001"&"001";
			--when TEST 	=> ctrl:=d_IR(3 downto 0)&"10"&"0"&"1"&"001"&"0010"&"001";
			when ORins	=> ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"001"&"0011"&"001";
			when XORins	=> ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"001"&"0100"&"001";
			when MOV 	=> ctrl:=d_IR(3 downto 0)&"10"&"1"&"1"&"011"&"0000"&"000";
			when LOADH	=> ctrl:="1111"&"10"&"1"&"1"&"101"&"1000"&"000";	
						   imm <=x"0"&d_IR(3 downto 0);
			when LOADL	=> ctrl:="1111"&"10"&"1"&"1"&"101"&"1001"&"000"; 
						   imm <=x"0"&d_IR(3 downto 0);
			when LOAD 	=> ctrl:=d_IR(3 downto 0)&"01"&"1"&"0"&"011"&"0000"&"000";		
			when JR|JNR =>  ctrl:=DoNothing; --若是JR*指令,计算offset，并向Exe插入Bubble
					            offset <= d_IR(3)&d_IR(3)&d_IR(3)&d_IR(3)&d_IR(3 downto 0);	    	
			when others => ctrl:=DoNothing;
		 end case;
		SA 		<= ctrl(17 downto 16);
		SB 		<= ctrl(15 downto 14);
		WrMem 	<= ctrl(13 downto 12);
		wRegEn 	<= ctrl(11);
		MemToReg<= ctrl(10);
		ALUSrc 	<= ctrl(9 downto 7);
		ALUOpr 	<= ctrl(6 downto 3);
		SetFlag <= ctrl(2 downto 0);
	end process;
	--*******************************************
	--计算跳转时的PC值	  		
	i_PCPlusOffset <= d_PCInc1 + offset;
	--*******************************************
	--流水控制信号传递 ID to Exe
	process(reset,clk)
	begin
		if reset = '0' then     
			e_wrMem	 <= "10";	-- do nothing		
			e_wRegEn <= '0'; 			
		elsif RISING_EDGE(clk) then
			e_SA <= SA;
			e_SB <= SB;
			e_RAOut <= RA;
			e_RBOut <= RB;
			e_IMM   <= imm;
			e_ALUSrc <= ALUSrc;
			e_ALUOpr <= ALUOpr;
			e_SetFlag <= SetFlag;
			e_wrMem  <= wrMem;
			e_wRegEn <= wRegEn;
			e_destReg <= SA;
			e_MemToReg <= memToReg;			
		end if;				
	end process;
	--*******************************************	
	RegOut <= RegArray(conv_integer(RegSel)); --寄存器输出
end architecture;
			
			
				
			
				
			