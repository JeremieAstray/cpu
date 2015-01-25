--MemAccessEntity.vhd- 访存模块
--------------------------------------
--选择地址线的数据来源和数据线的流向
--------------------------------------
--2004-09-04
library IEEE;
use IEEE.std_logic_1164.all; 
use IEEE.std_logic_unsigned.all;
use work.unitpack.all;

entity MemAccessEntity is
	port(	reset,clk	: in std_logic;
			m_wrMem		: in std_logic_vector(1 downto 0);
			w_wrMem     : out std_logic_vector(1 downto 0);
			m_ALUOut	: in word;    -- Memory addr
			m_RBdata	: in word;		
			wr			: out std_logic; --内存读写控制 '1':read , '0':write
			OuterDB		: inout word;
			w_ALUOut	: out word;	
			w_MemOut	: out word;	
			m_flag		: in  std_logic_vector(3 downto 0);	 
			w_flag		: out  std_logic_vector(3 downto 0); 
			PC          : in  word;  --PC输入
			addr        : out word;  --输出到内存地址线
			--*** for Forwarding	
			m_SA		: in std_logic_vector(3 downto 0);
			w_SA		: out std_logic_vector(3 downto 0);	 					
			--*** for WB Reg
			m_wRegEn  	: in std_logic; 
			m_destReg 	: in std_logic_vector(3 downto 0); 
			m_memToReg  : in std_logic;				
			--*** for WB Reg
			w_wRegEn 	: out std_logic; 
			w_destReg	: out std_logic_vector(3 downto 0); 
			w_memToReg  : out std_logic
		);
end entity ;

architecture MemAccessArch of MemAccessEntity is
begin 
	--内存读写控制***************************
	process(clk,m_wrMem,m_ALUOut,m_RBdata,PC)
	begin	 
		case m_wrMem is
			when "00" 	=>  addr <= m_ALUout;	
			                wr <= '0';  -- write Memory																											
							OuterDB <= m_RBdata;							
			when "01"   =>  wr <= '1';	
			                addr <= m_ALUout;
							OuterDB <= Z16; --MODIFY AT 09_04_20_45 PRE:M_RBDATA
			when others =>	wr <= '1';	  
			                addr <= PC;
			                OuterDB <= Z16;
		end case;
	end process;		
	--***************************************
	--控制信号Mem to WB
	process(reset,clk)
	begin
	    if reset='0' then
			w_wRegEn <= '0';
			w_wrMem  <= "10";
		elsif RISING_EDGE(clk) then  
			w_flag   <= m_flag;
			w_ALUOut <= m_ALUOut;
			w_SA     <= m_SA;
			w_wRegEn <= m_wRegEn;
			w_destReg<= m_destReg;
			w_memToReg<= m_memToReg;  
			w_memOut  <= OuterDB;
			w_wrMem  <= m_wrMem;
		end if;
	end process;
	--***************************************
end architecture ;
			
			
			