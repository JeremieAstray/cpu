--ForwardingEntity.vhd- 傍路处理模块
----------------------------------
--检测数据相关
----------------------------------
--2004-09-04
library IEEE;
use IEEE.std_logic_1164.all;   

entity ForwardingEntity	is
	port(	m_wRegEn		: in  std_logic;
			w_wRegEn		: in  std_logic;
			m_SA			: in  std_logic_vector(1 downto 0);
			w_SA			: in  std_logic_vector(1 downto 0);
			e_SA			: in  std_logic_vector(1 downto 0);
			e_SB			: in  std_logic_vector(1 downto 0);				
			forwardA		: out std_logic_vector(1 downto 0);	
			forwardB		: out std_logic_vector(1 downto 0)
		); 
end entity ;

architecture ForwardingArch of ForwardingEntity is
begin
	
	process(m_wRegEn,w_wRegEn,m_SA,w_sA,e_SA,e_SB)
	begin	
		--The first ALU operand comes from the prior ALU result			
		if m_wRegEn='1' and m_SA=e_SA    then
			 forwardA <= "10";
		--The first ALU operand comes from Memory or an earlier ALU result
		elsif w_wRegEn='1' and w_SA=e_SA then
			 forwardA <= "01";
		--The frist ALU operand comes from the register file
		else forwardA <= "00";
		end if;
		--The second ALU operand comes from the prior ALU result			
		if m_wRegEn='1' and m_SA=e_SB    then
			 forwardB <= "10";
		--The second ALU operand comes from Memory or an earlier ALU result
		elsif w_wRegEn='1' and w_SA=e_SB then
			 forwardB <= "01";
		--The second ALU operand comes from the register file
		else forwardB <= "00";
		end if;
	end process;
end architecture ;
