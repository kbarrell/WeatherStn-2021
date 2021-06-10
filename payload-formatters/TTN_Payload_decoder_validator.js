function Decoder(bytes, port) { 
  var windgust = ((bytes[1] << 8) | bytes[0])/10.0;
  var windgustdir = (bytes[3] << 8) | bytes[2];
  var temp = ((bytes[5] << 8) | bytes[4])/10.0 - 100.0;
  var humidity = ((bytes[7] << 8) | bytes[6])/10.0;
  var baropress = ((bytes[9] << 8) | bytes[8])/10.0;
  var rainfallrate = ((bytes[11] << 8) | bytes[10])/10.0;
  var windspeed = ((bytes[13] << 8) | bytes[12])/10.0;
  var winddirn = ((bytes[15] << 8) | bytes[14]) - 90.0;
  var dailyraintl = ((bytes[17] << 8) | bytes[16])/10.0;
  var casetemp = ((bytes[19] << 8) | bytes[18])/10.0 - 100.0;


  return {
    field1: windgust,
    field2: temp,
    field3: humidity,
    field4: baropress,
    field5: rainfallrate,
    field6: windspeed,
    field7: winddirn,
    field8: dailyraintl,
    field9: windgustdir,
    devicetemp: casetemp
  };
}

function Validator(converted, port) {
	// Return false if the decoded, converted
	// message is invalid and should be dropped.

  var checked = converted;
  
  if (port == 1 && checked.field3 === 0 && checked.field4 === 0) {
    return false;
  }
  return true;
}