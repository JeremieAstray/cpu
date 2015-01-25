--HazardDetectEntity.vhd- 结构相关处理模块
----------------------------------
--检测结构相关
----------------------------------
--2004-09-04
library IEEE;
use IEEE.std_logic_1164.all; 
use IEEE.std_logic_unsigned.all;
use work.unitpack.all;

entity HazardDetectEntity is
	port(	m_wrMem	: in std_logic_vector(1 downto 0);
			w_wrMem : in std_logic_vector(1 downto 0);
			d_IR	: in word;
			IFFlush	: out std_logic;
			PCStall : out std_logic
		);
end entity ;

architecture HazardDetectArch of HazardDetectEntity is
begin
	process(m_wrMem,d_IR)
	variable d_op : std_logic_vector(3 downto 0);
	begin
		d_op := d_IR(7 downto 4);
		case m_wrmem is
			when "00"|"01" => IFFlush <= '1';    --访存和取指冲突
							  if w_wrMem="00" or w_wrMem="01" then
							  	PCStall <= '1';
							  else
								PCStall <= '0';
							  end if;
			when others    => case d_op is       --访存和取指不冲突
								when NOP   => PCStall <='1';	
								  			  IFFlush <= '0';  										
								when others=> IFFlush <= '0';
								  			  PCStall <= '0'; 
							  end case;
	   end case;							
	end process;
end architecture ;