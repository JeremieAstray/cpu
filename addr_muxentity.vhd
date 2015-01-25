library IEEE;
use IEEE.std_logic_1164.all; 
use IEEE.std_logic_unsigned.all;
use work.unitpack.all;

entity Addr_muxEntity  is
	port(	PC	: in word;
		 	ALUOut : in word;
			addrSel: in std_logic;
			addr   : out word
		);
end entity ;

architecture Addr_muxArch of Addr_muxEntity is
begin			 
	with addrSel select
		addr <= PC		when '0',
		        ALUOut  when others;
end architecture;