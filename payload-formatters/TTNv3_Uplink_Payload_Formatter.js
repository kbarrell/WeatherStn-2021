function decodeUplink(input) { 
  var data = {};
  data.windgust = ((input.bytes[1] << 8) | input.bytes[0])/10.0;
  data.windgustdir = (input.bytes[3] << 8) | input.bytes[2];
  data.temp = ((input.bytes[5] << 8) | input.bytes[4])/10.0 - 100.0;
  data.humidity = ((input.bytes[7] << 8) | input.bytes[6])/10.0;
  data.baropress = ((input.bytes[9] << 8) | input.bytes[8])/10.0;
  data.rainfallrate = ((input.bytes[11] << 8) | input.bytes[10])/10.0;
  data.windspeed = ((input.bytes[13] << 8) | input.bytes[12])/10.0;
  data.winddirn = ((input.bytes[15] << 8) | input.bytes[14]) - 90.0;
  data.dailyraintl = ((input.bytes[17] << 8) | input.bytes[16])/10.0;
  data.casetemp = ((input.bytes[19] << 8) | input.bytes[18])/10.0 - 100.0;
  data.field1 = data.windgust;
  data.field2 = data.temp;
  data.field3 = data.humidity;
  data.field4 = data.baropress;
  data.field5 = data.rainfallrate;
  data.field6 = data.windspeed;
  data.field7 = data.winddirn;
  data.field8 = data.dailyraintl;
  data.field9 = data.windgustdir;
  data.devicetemp = data.casetemp;
  
  return {
    data: data
  };
}