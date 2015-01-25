--ExEntity.vhd- 执行模块
----------------------------------------------------------------------
--完成算术逻辑运算、计算有效地址和提供数据通道
----------------------------------------------------------------------
--2004-09-04
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use work.unitPack.all;

entity ExEntity is
	port(	reset,clk   		: in std_logic;
			--*** Input for ALU		        
			e_RAOut,e_RBOut		: in word;	 			
			e_ALUSrc			: in std_logic_vector(2 downto 0);
			e_ALUOpr			: in std_logic_vector(3 downto 0);
			e_SetFlag			: in std_logic_vector(2 downto 0);	
			e_IMM				: in word;			
		    forwardA,forwardB	: in  std_logic_vector(1 downto 0);--傍路选择	
			--*** for Forwarding  
			e_SA				: in std_logic_vector(3 downto 0);
			m_SA    			: out std_logic_vector(3 downto 0);				
			e_ALUOut  			: in word;
			w_WBdata  			: in word;			
			m_ALUOut			: out word;
			m_RBdata  			: out word;			
			i_tempZ	    		: out std_logic;
			i_tempC     		: out std_logic;
			--*** Control Signal for Ex_MemReg & Mem_WBReg 
			m_flag				: out std_logic_vector(3 downto 0);
			e_wRegEn  			: IN  std_logic; 
			m_wRegEn 			: out std_logic; 
			e_memToReg  		: in std_logic;
			m_memToReg  		: out std_logic;
			e_destReg 			: IN  std_logic_vector(3 downto 0); 
			m_destReg 			: out std_logic_vector(3 downto 0);			
			e_wrMem				: IN std_logic_vector(1 downto 0); --"00" write, "01" read, "1-" do nothing  			
			m_wrMem	  			: out std_logic_vector(1 downto 0) --"00" write, "01" read, "1-" do nothing					
		);
end entity ;  

architecture ExArch of ExEntity is
signal ALUaIn,ALUbIn : word;   
signal dout			 : word; 	   
signal C,Z,V,S		 : STD_LOGIC;
signal tempFlag		 : std_logic_vector(3 downto 0);  
begin
	--选择运算器A,B口数据来源 ******************
	with forwardA select
		ALUaIn	<= e_RAOut  when "00",
				   e_ALUOut when "10",
				   w_WBdata when "01",
				   e_RAOut  when others;
	with forwardB select
		ALUbIn	<= e_RBOut  when "00",
				   e_ALUOut when "10",
				   w_WBdata when "01",
				   e_RBOut  when others;
	--******************************************
	ALUActivity:process(ALUaIn,ALUbIn,e_ALUSrc,e_ALUOpr,e_setFlag,e_IMM,tempFlag)
	variable ALUResult,opR,opS : std_logic_vector(16 downto 0); -- 17 bits	
	variable cx,tempC,tempZ,tempV,tempS : std_logic; 	
	begin
		case e_ALUSrc is  --选择ALU的两入口数据
			when selA0	=>	opR := '0'&ALUaIn;	opS := ZERO17;
			when selAB	=>	opR := '0'&ALUaIn;	opS := '0'&ALUbIn;
			when selA1	=>	opR := '0'&ALUaIn;	opS := '0'&(X"0001");
            when sel0B	=>	opR := ZERO17	;	opS := '0'&ALUbIn;
			when selAF	=>	opR := '0'&ALUaIn;	opS := '1'&(X"FFFF");
			when selAD	=>	opR := '0'&ALUaIn;	opS := '0'&e_IMM;				
			when others =>  opR := ZERO17;		opS := ZERO17;
		end case;
		case e_ALUOpr is  --选择ALU的运算
			when aluAdd	  => ALUResult := opR + opS;
							 tempV := ((not opR(15))and(not opS(15))and ALUResult(15)) or (opR(15)and opS(15)and (not ALUResult(15)));  				
			when aluSub	  => ALUResult := opR - opS;
							 tempV := ( opS(15)and(not opR(15))and (not ALUResult(15))) or ((NOT opS(15))and opR(15)and ALUResult(15));  				
			when aluAnd	  => ALUResult := opR and opS;
			when aluOr	  => ALUResult := opR or  opS;
			when aluXor	  => ALUResult := opR xor opS;
			when aluShl	  => ALUResult(15 downto 1) := opR(14 downto 0);
							 ALUResult(0) := '0';	cx := opR(15);
			when aluShr	  => ALUResult(14 downto 0) := opR(15 downto 1);
							 ALUResult(15) := '0';	cx := opR(0); 
			when aluSar	  => ALUResult(14 downto 0) := opR(15 downto 1);
							 ALUResult(15) := opR(15); cx := opR(0);	
			when aluLOADH => ALUResult := opS(8 downto 0)&opR(7 downto 0); 
			when aluLOADL => ALUResult := '0'&opR(15 downto 8)&opS(7 downto 0);
			when others   => null;
		end case; 	
		dout <= ALUResult(15 downto 0);		
		------- Set Flag ------------------------------------------
		case e_ALUOpr is
			when aluAdd|aluSub            =>   tempC := ALUResult(16);			
			when aluShl|aluShr|aluSar     =>   tempC := cx;	
			when aluAnd|aluOr|aluXor 	  =>   tempC := '0'; tempV:= '0'; --逻辑运算,状态位C,Z置零
			when others					  =>   null;						
		end case;	
		i_tempC <= tempC;		
		if ALUResult = ZERO17 then	tempZ := '1';
		else tempZ := '0';	end if;	 
		i_tempZ <= tempZ;
		tempS := ALUResult(15);			
		case e_setFlag is
			when flag_hold	  => (C,Z,V,S)<= tempFLAG; 
			when flag_update  => C<=tempC;	Z <= tempZ;	V <= tempV;	S <= tempS;
		    when flag_InnerDB => (C,Z,V,S) <= e_IMM(3 downto 0);
			when flag_clear	  => C<='0'; Z<='0'; V<='0'; S<='0';
			when others		  => (C,Z,V,S)<= tempFLAG; 
		end case;	
	end process;
	--******************************************
	--修改状态寄存器
	WriteFlag:process(reset,clk)
	begin
		if reset = '0' then							
			tempFlag <= "0000";			
		elsif RISING_EDGE(clk) then
			tempFlag <= C&Z&V&S;  		
		end if;
		m_flag <= tempFlag;
	end process;   
	--******************************************
	--控制信号Ex to Mem
	process(reset,clk)
	begin  
		if reset='0' then 
			m_wrMem <= "10";
			m_wRegEn <= '0';
		elsif RISING_EDGE(clk) then
			m_SA <= e_SA;
			m_ALUOut <= dout;
			m_RBdata <= ALUbIn;
			m_wRegEn <= e_wRegEn; 
			m_memToReg <= e_memToReg;
			m_destReg  <= e_destReg;
			m_wrMem  <= e_wrMem;
	    end if;	
	end process;
end architecture ;