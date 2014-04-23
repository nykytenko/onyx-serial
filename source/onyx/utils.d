/**
 * Base utils
 *
 * Copyright: © 2014 onyx
 *
 * Authors: Oleg Nykytenko (onyx), onyx.itdevelopment@gmail.com
 *
 * Version: 4.xx
 *
 * Date: 10.02.2014
 *
 */

module onyx.utils;

import std.string;
import std.conv;


/**
 * Base utils for work with numbers
 */
struct OxNumberUtils {

	static int strToInt (string strNum){
		string sign = "";

		if (strNum.length == 0) return 0;
		
		if (strNum.length == 1) {
			if ((strNum[0] == '-') || (strNum[0] == '+') || (strNum[0] == '0')) return 0;
			return to!int(strNum); 
		}
		
		if (strNum[0] == '-'){
			sign = "-";
			strNum = strNum[1..$];
		}else if (strNum[0] == '+'){
			sign = "+";
			strNum = strNum[1..$];
		}

		if (strNum.length < 3) return to!int(sign ~ strNum);

		if ((strNum[0..2] == "0X") || (strNum[0..2] == "0x")) return to!int(strNum[2..$], 16);
		else if ((strNum[0..2] == "0B") || (strNum[0..2] == "0b")) return to!int(strNum[2..$], 2);
		else if ((strNum[0..1] == "0") || (strNum[0..1] == "0")) return to!int(strNum[2..$], 8);
		else return to!int(sign ~ strNum);
	}

	static double strToDouble (string strNum){
		if ((strNum.indexOf("Ox")>=0) || (strNum.indexOf("OX")>=0)) return cast(double)strToInt(strNum);
		return to!double(strNum);
	}
}

unittest{
	assert (OxNumberUtils.strToInt("0x22") == 0x22);
	assert (OxNumberUtils.strToInt("-21") == -21);
}