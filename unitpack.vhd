--UnitPack.vhd- 常量定义文件
--2004-09-04
library ieee;
use ieee.std_logic_1164.all;
PACKAGE unitPack IS	   
	constant ZERO16 : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00000000";	
	constant ZERO17 : STD_LOGIC_VECTOR(8 DOWNTO 0) := "000000000";
	constant Z16	: STD_LOGIC_VECTOR(7 DOWNTO 0) := "ZZZZZZZZ";	
	-- ALU: select data source
	constant selA0	: std_logic_vector(2 downto 0) := "000";
	constant selAB	: std_logic_vector(2 downto 0) := "001";
	constant selA1	: std_logic_vector(2 downto 0) := "010";
	constant sel0B	: std_logic_vector(2 downto 0) := "011";
	constant selAF	: std_logic_vector(2 downto 0) := "100";	
	constant selAD	: std_logic_vector(2 downto 0) := "101";
	-- ALU: Select operation
	constant aluAdd	: std_logic_vector(3 downto 0) := "0000";
	constant aluSub	: std_logic_vector(3 downto 0) := "0001"; 
	constant aluAnd	: std_logic_vector(3 downto 0) := "0010";
	constant aluOr 	: std_logic_vector(3 downto 0) := "0011"; 
	constant aluXor	: std_logic_vector(3 downto 0) := "0100";
	constant aluShl	: std_logic_vector(3 downto 0) := "0101";
	constant aluShr	: std_logic_vector(3 downto 0) := "0110";
	constant aluSar	: std_logic_vector(3 downto 0) := "0111"; 
	constant aluLOADH : std_logic_vector(3 downto 0) := "1000"; 
	constant aluLOADL : std_logic_vector(3 downto 0) := "1001"; 	
	-- 状态位的控制信号
	constant flag_hold    : std_logic_vector(2 downto 0)  := "000"; 
	constant flag_update  : std_logic_vector(2 downto 0)  := "001";
	constant flag_InnerDB : std_logic_vector(2 downto 0)  := "010"; 
	constant flag_C0	  : std_logic_vector(2 downto 0)  := "011";
	constant flag_C1	  : std_logic_vector(2 downto 0)  := "100";	
	constant flag_clear	  : std_logic_vector(2 downto 0)  := "101";
	-- 指令操作码
    --constant ADD	: std_logic_vector(7 downto 0):="00000000"; 
	constant ADD	: std_logic_vector(3 downto 0):="0000"; 
    --constant SUBB : std_logic_vector(7 downto 0):="00000001";
	constant SUBB 	: std_logic_vector(3 downto 0):="0001";
    --constant DEC 	: std_logic_vector(7 downto 0):="00000010";
   	constant DEC 	: std_logic_vector(3 downto 0):="0010";
    --constant INC 	: std_logic_vector(7 downto 0):="00000011";
   	constant INC 	: std_logic_vector(3 downto 0):="0011";
    --constant CMP 	: std_logic_vector(7 downto 0):="00000110";
   	constant CMP 	: std_logic_vector(3 downto 0):="0100"; 
    --constant ANDins : std_logic_vector(7 downto 0):="00000111"; 
   	constant ANDins : std_logic_vector(3 downto 0):="0101";
    --constant ORins 	: std_logic_vector(7 downto 0):="00001000";   
   	constant ORins 	: std_logic_vector(3 downto 0):="0110";
    --constant XORins : std_logic_vector(7 downto 0):="00001010"; 
   	constant XORins : std_logic_vector(3 downto 0):="0111";
    --constant TEST 	: std_logic_vector(7 downto 0):="00001011";  
   	constant JNR 	: std_logic_vector(3 downto 0):="1000";
   	--constant SHLIns	: std_logic_vector(7 downto 0):="00001100";
   	--constant SHRIns	: std_logic_vector(7 downto 0):="00001101";   
   	--constant SAR 	: std_logic_vector(7 downto 0):="00001110";
    --constant MOV 	: std_logic_vector(7 downto 0):="00001111";
   	constant MOV 	: std_logic_vector(3 downto 0):="1001";  
   	--constant LOAD : std_logic_vector(7 downto 0):="01000001"; 
    constant LOAD   : std_logic_vector(3 downto 0):="1110"; 
   	--constant STORE : std_logic_vector(7 downto 0):="01000010";
   	--constant PUSH : std_logic_vector(7 downto 0):="01000110";  
   	--constant POP 	: std_logic_vector(7 downto 0):="01000111";
    --constant JR 	: std_logic_vector(7 downto 0):="00010000"; 
   	constant JR 	: std_logic_vector(3 downto 0):="1010";
    --constant JRC 	: std_logic_vector(7 downto 0):="00010001"; 
   	--constant JRC 	: std_logic_vector(3 downto 0):="1011"; 
    --constant JRNC : std_logic_vector(7 downto 0):="00010010";
   	--constant JRNC 	: std_logic_vector(3 downto 0):="1100";
    --constant JRZ 	: std_logic_vector(7 downto 0):="00010011"; 
   	--constant JRZ 	: std_logic_vector(3 downto 0):="1101";
    --constant JRNZ 	: std_logic_vector(7 downto 0):="00010100"; 
   	--constant JRNZ 	: std_logic_vector(3 downto 0):="1110";
    --constant NOP 	: std_logic_vector(7 downto 0):="11000000"; 
   	constant NOP 	: std_logic_vector(3 downto 0):="1111";	
	--constant LOADH 	: std_logic_vector(7 downto 0):="00100000";	 
	constant LOADH 	: std_logic_vector(3 downto 0):="1100";	 
	--constant LOADL 	: std_logic_vector(7 downto 0):="00100001";
	constant LOADL 	: std_logic_vector(3 downto 0):="1101";
	constant NOPIns	: std_logic_vector(7 downto 0):="11000000";  	
	-- 空操作控制信号
	constant DoNothing :std_logic_vector(17 downto 0):="0000"&"10"&"0"&"1"&"011"&"0000"&"000";
	-- Type definition
	subtype INDEXTYPE is INTEGER range 0 to 7;
 	subtype WORD is std_logic_vector(7 downto 0); 
    type REGISTERARRAY is array ( 0 to 7 ) of WORD;
END unitPack;





